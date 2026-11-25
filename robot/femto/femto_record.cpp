// femto_record: record depth, IR and colour from every Orbbec Femto Bolt on the robot through
// Orbbec SDK v2, with each device's factory calibration. Built and run on the Jetson by
// `just femto-record` (robot/femto/femto-record.sh). Moves nothing.
//
// Layout of <out>/<serial>/:
//   calibration.json          serial, firmware, USB port, sync config, intrinsics/distortion of the
//                             recorded depth/IR/colour profiles, their extrinsics (mm), and every
//                             factory camera-param group the device reports
//   depth/NNNNNN.png          16-bit depth in mm (PNG, lossless)
//   ir/NNNNNN.png             16-bit IR (every --ir-every'th frame only)
//   color/NNNNNN.jpg          the camera's own MJPEG bytes, not re-encoded
//   aligned/NNNNNN_depth.png  depth aligned to colour (1280x720), a few samples
//   index_<stream>.csv        one row per frame the SDK delivered: our sequence, SDK frame index,
//                             device timestamp (us), SDK system timestamp (us), global timestamp
//                             (us), host CLOCK_MONOTONIC and CLOCK_REALTIME at callback (ns),
//                             bytes, file ("" if not written, with the reason in `note`)
// <out>/summary.json: frame counts, SDK frame-index gaps and writer-queue drops per stream.
// Nothing is dropped silently: every delivered frame has an index row, every gap is counted.

#include <libobsensor/ObSensor.hpp>
#include <png.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <deque>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <vector>

namespace {

int64_t clockNs(clockid_t id) {
    timespec ts{};
    clock_gettime(id, &ts);
    return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

void mkdirs(const std::string &p) {
    std::string acc;
    std::stringstream ss(p);
    std::string part;
    if(!p.empty() && p[0] == '/') acc = "/";
    while(std::getline(ss, part, '/')) {
        if(part.empty()) continue;
        acc += part + "/";
        mkdir(acc.c_str(), 0755);
    }
}

bool writePng16(const std::string &path, const uint8_t *data, int w, int h) {
    FILE *fp = fopen(path.c_str(), "wb");
    if(!fp) return false;
    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    png_infop   info = png ? png_create_info_struct(png) : nullptr;
    if(!png || !info || setjmp(png_jmpbuf(png))) {
        if(png) png_destroy_write_struct(&png, info ? &info : nullptr);
        fclose(fp);
        return false;
    }
    png_init_io(png, fp);
    png_set_compression_level(png, 1);
    png_set_filter(png, 0, PNG_FILTER_SUB);
    png_set_IHDR(png, info, w, h, 16, PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png, info);
    png_set_swap(png);  // our buffer is little-endian; PNG is big-endian
    for(int y = 0; y < h; ++y) png_write_row(png, const_cast<png_bytep>(data + size_t(y) * w * 2));
    png_write_end(png, nullptr);
    png_destroy_write_struct(&png, &info);
    return fclose(fp) == 0;
}

bool writeRaw(const std::string &path, const uint8_t *data, size_t n) {
    FILE *fp = fopen(path.c_str(), "wb");
    if(!fp) return false;
    bool ok = fwrite(data, 1, n, fp) == n;
    return (fclose(fp) == 0) && ok;
}

struct Job {
    std::string          path;
    std::vector<uint8_t> data;
    int                  w = 0, h = 0;
    bool                 png16 = false;
};

// Bounded writer pool: a full queue drops the frame and counts it (reported per stream).
class Writer {
public:
    Writer(size_t cap, int threads) : cap_(cap) {
        for(int i = 0; i < threads; ++i) th_.emplace_back([this] { run(); });
    }
    ~Writer() { finish(); }
    bool push(Job &&j) {
        std::lock_guard<std::mutex> lk(m_);
        if(q_.size() >= cap_) return false;
        q_.push_back(std::move(j));
        maxDepth_ = std::max(maxDepth_, q_.size());
        cv_.notify_one();
        return true;
    }
    void finish() {
        {
            std::lock_guard<std::mutex> lk(m_);
            if(done_) return;
            done_ = true;
        }
        cv_.notify_all();
        for(auto &t: th_) t.join();
    }
    size_t failures() const { return failures_; }
    size_t maxDepth() const { return maxDepth_; }

private:
    void run() {
        for(;;) {
            Job j;
            {
                std::unique_lock<std::mutex> lk(m_);
                cv_.wait(lk, [this] { return done_ || !q_.empty(); });
                if(q_.empty()) return;
                j = std::move(q_.front());
                q_.pop_front();
            }
            bool ok = j.png16 ? writePng16(j.path, j.data.data(), j.w, j.h) : writeRaw(j.path, j.data.data(), j.data.size());
            if(!ok) {
                failures_++;
                std::cerr << "write failed: " << j.path << std::endl;
            }
        }
    }
    size_t                   cap_;
    std::mutex               m_;
    std::condition_variable  cv_;
    std::deque<Job>          q_;
    std::vector<std::thread> th_;
    bool                     done_ = false;
    std::atomic<size_t>      failures_{0};
    size_t                   maxDepth_ = 0;
};

struct StreamStats {
    uint64_t delivered = 0, written = 0, skipped = 0, queueDrops = 0, indexGaps = 0, missedFrames = 0;
    int64_t  lastIndex = -1;
    uint64_t firstDevUs = 0, lastDevUs = 0;
    int64_t  firstMonoNs = 0, lastMonoNs = 0;
};

struct Args {
    std::string out;
    double      seconds = 120;
    int         fps = 5;
    int         depthW = 640, depthH = 576;
    int         colorW = 1280, colorH = 720;
    bool        ir = true;
    int         irEvery = 5;
    int         alignedSamples = 3;
    std::string syncMode = "keep";  // keep | standalone | free-run
    std::string portMap;            // "serial=label,serial=label"
    bool        probeOnly = false;
};

std::string jsonEsc(const std::string &s) {
    std::string o;
    for(char c: s) {
        if(c == '"' || c == '\\') o += '\\';
        o += c;
    }
    return o;
}

std::string intrJson(const OBCameraIntrinsic &i) {
    std::ostringstream o;
    o.precision(9);
    o << "{\"fx\":" << i.fx << ",\"fy\":" << i.fy << ",\"cx\":" << i.cx << ",\"cy\":" << i.cy << ",\"width\":" << i.width << ",\"height\":" << i.height << "}";
    return o.str();
}

std::string distJson(const OBCameraDistortion &d) {
    std::ostringstream o;
    o.precision(9);
    o << "{\"k1\":" << d.k1 << ",\"k2\":" << d.k2 << ",\"k3\":" << d.k3 << ",\"k4\":" << d.k4 << ",\"k5\":" << d.k5 << ",\"k6\":" << d.k6 << ",\"p1\":" << d.p1
      << ",\"p2\":" << d.p2 << ",\"model\":" << int(d.model) << "}";
    return o.str();
}

std::string extrJson(const OBExtrinsic &e) {
    std::ostringstream o;
    o.precision(9);
    o << "{\"rot\":[";
    for(int i = 0; i < 9; ++i) o << (i ? "," : "") << e.rot[i];
    o << "],\"trans_mm\":[";
    for(int i = 0; i < 3; ++i) o << (i ? "," : "") << e.trans[i];
    o << "]}";
    return o.str();
}

std::string syncJson(const OBMultiDeviceSyncConfig &c) {
    std::ostringstream o;
    o << "{\"syncMode\":" << int(c.syncMode) << ",\"depthDelayUs\":" << c.depthDelayUs << ",\"colorDelayUs\":" << c.colorDelayUs
      << ",\"trigger2ImageDelayUs\":" << c.trigger2ImageDelayUs << ",\"triggerOutEnable\":" << (c.triggerOutEnable ? "true" : "false")
      << ",\"triggerOutDelayUs\":" << c.triggerOutDelayUs << ",\"framesPerTrigger\":" << c.framesPerTrigger << "}";
    return o.str();
}

struct Cam {
    std::string                  serial, label, dir, uid, fw, conn;
    std::shared_ptr<ob::Device>   dev;
    std::shared_ptr<ob::Pipeline> pipe;
    std::shared_ptr<ob::Config>   cfg;
    std::shared_ptr<ob::Align>    align;
    std::map<std::string, StreamStats>    stats;
    std::map<std::string, std::ofstream*> idx;
    std::mutex                    m;
    uint64_t                      framesets = 0;
    int                           alignedDone = 0;
    std::string                   syncBefore, syncAfter;
    bool                          globalTs = false;
};

std::atomic<bool> g_recording{false};
int64_t           g_startMono = 0;

void onFrame(Cam &c, Writer &w, const Args &a, const std::string &name, std::shared_ptr<ob::Frame> f, int64_t mono, int64_t real) {
    auto &s = c.stats[name];
    uint64_t seq = s.delivered++;
    int64_t  fi  = int64_t(f->getIndex());
    if(s.lastIndex >= 0 && fi != s.lastIndex + 1) {
        s.indexGaps++;
        if(fi > s.lastIndex) s.missedFrames += uint64_t(fi - s.lastIndex - 1);
    }
    s.lastIndex = fi;
    uint64_t dev = f->getTimeStampUs(), sys = f->getSystemTimeStampUs(), glb = 0;
    try { glb = f->getGlobalTimeStampUs(); } catch(...) {}
    if(s.firstMonoNs == 0) { s.firstMonoNs = mono; s.firstDevUs = dev; }
    s.lastMonoNs = mono; s.lastDevUs = dev;

    char        fn[64];
    std::string file, note;
    bool        want = true;
    if(name == "ir" && a.irEvery > 1 && (seq % uint64_t(a.irEvery)) != 0) { want = false; note = "ir-every"; }
    if(want) {
        Job j;
        auto vf = f->as<ob::VideoFrame>();
        if(name == "color") {
            snprintf(fn, sizeof fn, "color/%06llu.%s", (unsigned long long)seq, f->getFormat() == OB_FORMAT_MJPG ? "jpg" : "raw");
        } else {
            snprintf(fn, sizeof fn, "%s/%06llu.png", name.c_str(), (unsigned long long)seq);
            j.png16 = true;
            j.w = int(vf->getWidth());
            j.h = int(vf->getHeight());
        }
        j.path = c.dir + "/" + fn;
        j.data.assign(f->getData(), f->getData() + f->getDataSize());
        if(w.push(std::move(j))) { file = fn; s.written++; }
        else { s.queueDrops++; note = "writer-queue-full"; }
    } else {
        s.skipped++;
    }
    auto *o = c.idx[name];
    *o << seq << ',' << fi << ',' << dev << ',' << sys << ',' << glb << ',' << mono << ',' << real << ',' << f->getDataSize() << ',' << file << ',' << note
       << '\n';
}

void onFrameset(Cam &c, Writer &w, const Args &a, std::shared_ptr<ob::FrameSet> fs) {
    int64_t mono = clockNs(CLOCK_MONOTONIC), real = clockNs(CLOCK_REALTIME);
    if(!g_recording) return;
    std::lock_guard<std::mutex> lk(c.m);
    c.framesets++;
    auto d = fs->getFrame(OB_FRAME_DEPTH), col = fs->getFrame(OB_FRAME_COLOR), ir = fs->getFrame(OB_FRAME_IR);
    if(d) onFrame(c, w, a, "depth", d, mono, real);
    if(col) onFrame(c, w, a, "color", col, mono, real);
    if(ir) onFrame(c, w, a, "ir", ir, mono, real);
    // A few depth-aligned-to-colour samples, spread over the recording.
    if(d && col && c.align && c.alignedDone < a.alignedSamples) {
        double t       = (mono - g_startMono) / 1e9;
        double target  = a.seconds * (c.alignedDone + 0.5) / a.alignedSamples;
        if(t >= target) {
            try {
                auto out = c.align->process(fs);
                auto ofs = out ? out->as<ob::FrameSet>() : nullptr;
                auto ad  = ofs ? ofs->getFrame(OB_FRAME_DEPTH) : nullptr;
                if(ad) {
                    auto vf = ad->as<ob::VideoFrame>();
                    char fn[96];
                    uint64_t cseq = c.stats["color"].delivered - 1;
                    snprintf(fn, sizeof fn, "/aligned/%06llu_depth_%ux%u.png", (unsigned long long)cseq, vf->getWidth(), vf->getHeight());
                    writePng16(c.dir + fn, ad->getData(), int(vf->getWidth()), int(vf->getHeight()));
                    snprintf(fn, sizeof fn, "/aligned/%06llu_color.jpg", (unsigned long long)cseq);
                    writeRaw(c.dir + fn, col->getData(), col->getDataSize());
                    std::cout << "[" << c.label << "] aligned sample " << c.alignedDone << " at color seq " << cseq << std::endl;
                }
            } catch(ob::Error &e) {
                std::cerr << "[" << c.label << "] align failed: " << e.what() << std::endl;
            }
            c.alignedDone++;
        }
    }
}

void writeCalibration(Cam &c, const Args &a) {
    std::ofstream o(c.dir + "/calibration.json");
    o.precision(9);
    o << "{\n  \"serial\":\"" << jsonEsc(c.serial) << "\",\n  \"label\":\"" << jsonEsc(c.label) << "\",\n  \"uid\":\"" << jsonEsc(c.uid)
      << "\",\n  \"firmware\":\"" << jsonEsc(c.fw) << "\",\n  \"connection\":\"" << jsonEsc(c.conn) << "\",\n  \"global_timestamp\":" << (c.globalTs ? "true" : "false")
      << ",\n  \"sync_config_before\":" << (c.syncBefore.empty() ? "null" : c.syncBefore) << ",\n  \"sync_config_used\":" << (c.syncAfter.empty() ? "null" : c.syncAfter)
      << ",\n  \"units\":{\"depth\":\"mm\",\"extrinsic_trans\":\"mm\",\"timestamps\":\"us (device), ns (host clocks)\"},\n  \"streams\":{";
    std::map<std::string, std::shared_ptr<ob::VideoStreamProfile>> profs;
    auto list = c.cfg->getEnabledStreamProfileList();
    for(uint32_t i = 0; i < list->getCount(); ++i) {
        auto p = list->getProfile(i);
        std::string n = p->getType() == OB_STREAM_DEPTH ? "depth" : p->getType() == OB_STREAM_COLOR ? "color" : p->getType() == OB_STREAM_IR ? "ir" : "other";
        if(n == "other") continue;
        profs[n] = p->as<ob::VideoStreamProfile>();
    }
    bool first = true;
    for(auto &kv: profs) {
        auto &p = kv.second;
        o << (first ? "" : ",") << "\n    \"" << kv.first << "\":{\"width\":" << p->getWidth() << ",\"height\":" << p->getHeight() << ",\"fps\":" << p->getFps()
          << ",\"format\":" << int(p->getFormat()) << ",\"intrinsic\":" << intrJson(p->getIntrinsic()) << ",\"distortion\":" << distJson(p->getDistortion()) << "}";
        first = false;
    }
    o << "\n  },\n  \"extrinsics\":{";
    first = true;
    for(auto &from: profs)
        for(auto &to: profs) {
            if(from.first == to.first) continue;
            try {
                auto e = from.second->getExtrinsicTo(to.second);
                o << (first ? "" : ",") << "\n    \"" << from.first << "_to_" << to.first << "\":" << extrJson(e);
                first = false;
            } catch(ob::Error &) {}
        }
    o << "\n  },\n  \"factory_camera_params\":[";
    try {
        auto pl = c.dev->getCalibrationCameraParamList();
        for(uint32_t i = 0; i < pl->getCount(); ++i) {
            auto cp = pl->getCameraParam(i);
            o << (i ? "," : "") << "\n    {\"depthIntrinsic\":" << intrJson(cp.depthIntrinsic) << ",\"rgbIntrinsic\":" << intrJson(cp.rgbIntrinsic)
              << ",\"depthDistortion\":" << distJson(cp.depthDistortion) << ",\"rgbDistortion\":" << distJson(cp.rgbDistortion)
              << ",\"depth_to_rgb\":" << extrJson(cp.transform) << ",\"isMirrored\":" << (cp.isMirrored ? "true" : "false") << "}";
        }
    } catch(ob::Error &e) {
        std::cerr << "[" << c.label << "] calibration list: " << e.what() << std::endl;
    }
    o << "\n  ]\n}\n";
}

bool parseArgs(int argc, char **argv, Args &a) {
    for(int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        auto        v = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
        if(k == "--out") a.out = v();
        else if(k == "--seconds") a.seconds = std::stod(v());
        else if(k == "--fps") a.fps = std::stoi(v());
        else if(k == "--depth") { std::string s = v(); sscanf(s.c_str(), "%dx%d", &a.depthW, &a.depthH); }
        else if(k == "--color") { std::string s = v(); sscanf(s.c_str(), "%dx%d", &a.colorW, &a.colorH); }
        else if(k == "--ir-every") { a.irEvery = std::stoi(v()); a.ir = a.irEvery > 0; }
        else if(k == "--aligned") a.alignedSamples = std::stoi(v());
        else if(k == "--sync") a.syncMode = v();
        else if(k == "--labels") a.portMap = v();
        else if(k == "--probe") a.probeOnly = true;
        else { std::cerr << "unknown arg " << k << std::endl; return false; }
    }
    if(a.out.empty() && !a.probeOnly) { std::cerr << "--out required" << std::endl; return false; }
    return true;
}

}  // namespace

int main(int argc, char **argv) {
    Args a;
    if(!parseArgs(argc, argv, a)) return 2;
    std::map<std::string, std::string> labels;
    {
        std::stringstream ss(a.portMap);
        std::string       kv;
        while(std::getline(ss, kv, ',')) {
            auto eq = kv.find('=');
            if(eq != std::string::npos) labels[kv.substr(0, eq)] = kv.substr(eq + 1);
        }
    }
    try {
        ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_WARN);
        ob::Context ctx;
        auto        list = ctx.queryDeviceList();
        std::cout << "devices: " << list->getCount() << std::endl;
        std::vector<std::unique_ptr<Cam>> cams;
        for(uint32_t i = 0; i < list->getCount(); ++i) {
            auto dev  = list->getDevice(i);
            auto info = dev->getDeviceInfo();
            std::unique_ptr<Cam> c(new Cam);
            c->dev    = dev;
            c->serial = info->getSerialNumber();
            c->uid    = info->getUid();
            c->fw     = info->getFirmwareVersion();
            c->conn   = info->getConnectionType();
            c->label  = labels.count(c->serial) ? labels[c->serial] : c->serial;
            std::cout << "device " << i << ": " << info->getName() << " serial=" << c->serial << " label=" << c->label << " fw=" << c->fw << " conn=" << c->conn
                      << " uid=" << c->uid << std::endl;
            try {
                auto sc       = dev->getMultiDeviceSyncConfig();
                c->syncBefore = syncJson(sc);
                std::cout << "  sync config: " << c->syncBefore << " supported bitmap=0x" << std::hex << dev->getSupportedMultiDeviceSyncModeBitmap() << std::dec
                          << std::endl;
                if(a.syncMode != "keep") {
                    sc.syncMode = a.syncMode == "free-run" ? OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN : OB_MULTI_DEVICE_SYNC_MODE_STANDALONE;
                    if(!a.probeOnly) dev->setMultiDeviceSyncConfig(sc);
                }
                c->syncAfter = syncJson(sc);
            } catch(ob::Error &e) {
                std::cerr << "  sync config: " << e.what() << std::endl;
            }
            try {
                c->globalTs = dev->isGlobalTimestampSupported();
                if(c->globalTs && !a.probeOnly) dev->enableGlobalTimestamp(true);
                std::cout << "  global timestamp supported: " << c->globalTs << std::endl;
            } catch(ob::Error &e) {
                std::cerr << "  global timestamp: " << e.what() << std::endl;
            }
            cams.push_back(std::move(c));
        }
        if(a.probeOnly) return cams.empty() ? 1 : 0;
        if(cams.empty()) { std::cerr << "no devices" << std::endl; return 1; }

        Writer w(400, 3);
        for(auto &cp: cams) {
            Cam &c = *cp;
            c.dir  = a.out + "/" + c.label;
            for(auto s: {"depth", "color", "ir", "aligned"}) mkdirs(c.dir + "/" + s);
            for(auto s: {"depth", "color", "ir"}) {
                c.idx[s] = new std::ofstream(c.dir + "/index_" + s + ".csv");
                *c.idx[s] << "seq,sdk_index,device_ts_us,system_ts_us,global_ts_us,host_mono_ns,host_realtime_ns,bytes,file,note\n";
            }
            c.pipe = std::make_shared<ob::Pipeline>(c.dev);
            c.cfg  = std::make_shared<ob::Config>();
            c.cfg->enableVideoStream(OB_STREAM_DEPTH, a.depthW, a.depthH, a.fps, OB_FORMAT_Y16);
            c.cfg->enableVideoStream(OB_STREAM_COLOR, a.colorW, a.colorH, a.fps, OB_FORMAT_MJPG);
            if(a.ir) c.cfg->enableVideoStream(OB_STREAM_IR, a.depthW, a.depthH, a.fps, OB_FORMAT_Y16);
            c.cfg->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);
            try { c.pipe->enableFrameSync(); } catch(ob::Error &e) { std::cerr << "frame sync: " << e.what() << std::endl; }
            c.align = std::make_shared<ob::Align>(OB_STREAM_COLOR);
        }
        g_startMono = clockNs(CLOCK_MONOTONIC);
        for(auto &cp: cams) {
            Cam *c = cp.get();
            c->pipe->start(c->cfg, [c, &w, &a](std::shared_ptr<ob::FrameSet> fs) { onFrameset(*c, w, a, fs); });
            std::cout << "[" << c->label << "] started" << std::endl;
            writeCalibration(*c, a);
        }
        g_startMono = clockNs(CLOCK_MONOTONIC);
        g_recording = true;
        auto t0     = std::chrono::steady_clock::now();
        int  tick   = 0;
        while(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < a.seconds) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            if(++tick % 20 == 0) {
                for(auto &cp: cams) {
                    std::lock_guard<std::mutex> lk(cp->m);
                    std::cout << "[" << cp->label << "] t=" << tick / 2 << "s framesets=" << cp->framesets;
                    for(auto &kv: cp->stats) std::cout << " " << kv.first << "=" << kv.second.delivered << "(miss " << kv.second.missedFrames << ", qdrop " << kv.second.queueDrops << ")";
                    std::cout << std::endl;
                }
            }
        }
        g_recording = false;
        for(auto &cp: cams) cp->pipe->stop();
        w.finish();
        std::ofstream sum(a.out + "/summary.json");
        sum << "{\"seconds\":" << a.seconds << ",\"fps\":" << a.fps << ",\"ir_every\":" << a.irEvery << ",\"writer_failures\":" << w.failures()
            << ",\"writer_max_queue\":" << w.maxDepth() << ",\"cameras\":{";
        bool firstCam = true;
        for(auto &cp: cams) {
            for(auto &kv: cp->idx) { kv.second->flush(); delete kv.second; }
            sum << (firstCam ? "" : ",") << "\n \"" << cp->label << "\":{\"serial\":\"" << cp->serial << "\",\"framesets\":" << cp->framesets << ",\"streams\":{";
            bool first = true;
            for(auto &kv: cp->stats) {
                auto &s = kv.second;
                sum << (first ? "" : ",") << "\"" << kv.first << "\":{\"delivered\":" << s.delivered << ",\"written\":" << s.written << ",\"skipped\":" << s.skipped
                    << ",\"queue_drops\":" << s.queueDrops << ",\"index_gaps\":" << s.indexGaps << ",\"missed_frames\":" << s.missedFrames
                    << ",\"first_device_ts_us\":" << s.firstDevUs << ",\"last_device_ts_us\":" << s.lastDevUs << ",\"first_host_mono_ns\":" << s.firstMonoNs
                    << ",\"last_host_mono_ns\":" << s.lastMonoNs << "}";
                first = false;
                std::cout << "[" << cp->label << "] " << kv.first << ": delivered " << s.delivered << ", written " << s.written << ", skipped " << s.skipped
                          << ", queue drops " << s.queueDrops << ", missed (sdk index gaps) " << s.missedFrames << std::endl;
            }
            sum << "}}";
            firstCam = false;
        }
        sum << "}}\n";
        std::cout << "writer failures " << w.failures() << ", max queue " << w.maxDepth() << std::endl;
        return w.failures() ? 3 : 0;
    } catch(ob::Error &e) {
        std::cerr << "ob::Error: " << e.what() << std::endl;
        return 1;
    }
}
