namespace Teleop.CameraHost
{
    /// <summary>
    /// The robot-side camera sender of docs/adr/0014-camera-frame-downlink.md, plus the tools used to
    /// prove it:
    /// <list type="bullet">
    /// <item><c>serve</c>: stream the camera to whoever subscribes (Linux, on the Jetson).</item>
    /// <item><c>probe</c>: subscribe and report what arrives (any OS; the operator's machine).</item>
    /// <item><c>timestamp-spike</c>: the ADR's first implementation step (Linux).</item>
    /// </list>
    /// </summary>
    internal static class Program
    {
        private const int WarmUp = 5;

        private const string Usage =
            "Usage:\n" +
            "  Teleop.CameraHost serve [--device /dev/video0] [--width 640] [--height 480] [--fps 30]\n" +
            "                          [--port 6003] [--subscriber-timeout 3] [--seconds 0]\n" +
            "      Stream native MJPEG frames to the latest subscriber (docs/adr/0014). --seconds 0 runs\n" +
            "      until Ctrl+C or SIGTERM. Linux only.\n" +
            "  Teleop.CameraHost probe --host <robot ip> [--port 6003] [--local-port 6004] [--seconds 10]\n" +
            "                          [--max-fps 0] [--save <path.jpg>]\n" +
            "      Subscribe, reassemble and report what arrives. Exit 0 if a frame arrived, else 1.\n" +
            "  Teleop.CameraHost timestamp-spike [--device /dev/video0] [--width 640] [--height 480]\n" +
            "                          [--fps 30] [--frames 150]\n" +
            "      Check the driver's capture timestamps against Stopwatch. Linux only.\n" +
            "Exit codes: 0 success, 1 check failed, 2 usage or setup error.";

        private static int Main(string[] args)
        {
            string verb = args.Length > 0 ? args[0] : string.Empty;
            if (verb != "serve" && verb != "probe" && verb != "timestamp-spike")
            {
                Console.Error.WriteLine(Usage);
                return 2;
            }

            if (verb != "probe" && !OperatingSystem.IsLinux())
            {
                Console.Error.WriteLine($"error: {verb} needs V4L2, so Linux; run it on the Jetson.");
                return 2;
            }

            string device = "/dev/video0", host = string.Empty;
            string? save = null;
            uint width = 640, height = 480, fps = 30;
            int frames = 150, port = 6003, localPort = 6004;
            double seconds = verb == "probe" ? 10 : 0, subscriberTimeout = 3;
            ushort maxFps = 0;

            for (int i = 1; i < args.Length; i++)
            {
                bool hasValue = i + 1 < args.Length;
                string value = hasValue ? args[i + 1] : string.Empty;
                bool ok = args[i] switch
                {
                    "--device" when hasValue => Set(ref device, value),
                    "--host" when hasValue => Set(ref host, value),
                    "--save" when hasValue => Set(ref save, value),
                    "--width" when hasValue => uint.TryParse(value, out width),
                    "--height" when hasValue => uint.TryParse(value, out height),
                    "--fps" when hasValue => uint.TryParse(value, out fps),
                    "--frames" when hasValue => int.TryParse(value, out frames),
                    "--port" when hasValue => int.TryParse(value, out port),
                    "--local-port" when hasValue => int.TryParse(value, out localPort),
                    "--seconds" when hasValue => double.TryParse(value, out seconds),
                    "--subscriber-timeout" when hasValue => double.TryParse(value, out subscriberTimeout),
                    "--max-fps" when hasValue => ushort.TryParse(value, out maxFps),
                    _ => false,
                };

                if (!ok)
                {
                    Console.Error.WriteLine($"error: unrecognised or invalid argument '{args[i]}'");
                    Console.Error.WriteLine(Usage);
                    return 2;
                }

                i++;
            }

            if (width == 0 || height == 0 || fps == 0 || port is <= 0 or > 65535 || localPort is <= 0 or > 65535 ||
                seconds < 0 || subscriberTimeout <= 0)
            {
                Console.Error.WriteLine("error: sizes, fps, ports and the subscriber timeout must be positive; seconds must not be negative.");
                return 2;
            }

            switch (verb)
            {
                case "serve":
                    return ServeCommand.Run(device, width, height, fps, port, subscriberTimeout, seconds);
                case "probe":
                    if (host.Length == 0 || seconds == 0)
                    {
                        Console.Error.WriteLine("error: probe needs --host and a positive --seconds.");
                        return 2;
                    }

                    return ProbeCommand.Run(host, port, localPort, seconds, maxFps, save);
                default:
                    if (frames <= WarmUp)
                    {
                        Console.Error.WriteLine($"error: --frames must be more than {WarmUp}.");
                        return 2;
                    }

                    return TimestampSpike.Run(device, width, height, fps, frames);
            }
        }

        private static bool Set<T>(ref T target, T value)
        {
            target = value;
            return true;
        }
    }
}
