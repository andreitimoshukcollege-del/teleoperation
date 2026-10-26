using System.Net;
using System.Net.Sockets;

namespace Teleop.CameraHost.Viewing
{
    /// <summary>
    /// Serves the newest camera frame to a browser on this machine only (127.0.0.1): <c>/</c> the page,
    /// <c>/stream</c> the frames as MJPEG, <c>/stats</c> one status line. Each client gets its own
    /// thread and only ever the newest frame, so a slow window skips frames instead of falling behind.
    /// </summary>
    internal sealed class MjpegServer : IDisposable
    {
        private readonly TcpListener _listener;
        private readonly string _title;
        private readonly object _gate = new();
        private byte[]? _frame;
        private long _frameNumber;
        private string _stats = "waiting for the camera...";
        private volatile bool _stopped;

        public MjpegServer(int port, string title)
        {
            _title = title;
            _listener = new TcpListener(IPAddress.Loopback, port);
            _listener.Start();
            Port = ((IPEndPoint)_listener.LocalEndpoint).Port;
            new Thread(AcceptLoop) { IsBackground = true, Name = "mjpeg-accept" }.Start();
        }

        public int Port { get; }

        public string Url => $"http://localhost:{Port}/";

        /// <summary>Hands over a frame; the server keeps the array, so pass a fresh one each time.</summary>
        public void Publish(byte[] jpeg)
        {
            lock (_gate)
            {
                _frame = jpeg;
                _frameNumber++;
                Monitor.PulseAll(_gate);
            }
        }

        public void SetStats(string line)
        {
            lock (_gate)
            {
                _stats = line;
            }
        }

        public void Dispose()
        {
            _stopped = true;
            _listener.Stop();
            lock (_gate)
            {
                Monitor.PulseAll(_gate);
            }
        }

        private void AcceptLoop()
        {
            while (!_stopped)
            {
                TcpClient client;
                try
                {
                    client = _listener.AcceptTcpClient();
                }
                catch (Exception e) when (e is SocketException || e is ObjectDisposedException || e is InvalidOperationException)
                {
                    return;
                }

                new Thread(() => Serve(client)) { IsBackground = true, Name = "mjpeg-client" }.Start();
            }
        }

        private void Serve(TcpClient client)
        {
            using (client)
            {
                try
                {
                    client.NoDelay = true;
                    NetworkStream stream = client.GetStream();
                    var reader = new StreamReader(stream);
                    string? path = MjpegHttp.RequestPath(reader.ReadLine());
                    while (!string.IsNullOrEmpty(reader.ReadLine()))
                    {
                        // The rest of the request's headers: nothing here needs them.
                    }

                    switch (path)
                    {
                        case "/":
                            Write(stream, MjpegHttp.Response("200 OK", "text/html; charset=utf-8", MjpegHttp.Page(_title)));
                            break;
                        case "/stats":
                            string stats;
                            lock (_gate)
                            {
                                stats = _stats;
                            }

                            Write(stream, MjpegHttp.Response("200 OK", "text/plain; charset=utf-8", stats));
                            break;
                        case "/stream":
                            Stream(stream);
                            break;
                        default:
                            Write(stream, MjpegHttp.Response("404 Not Found", "text/plain", "not found"));
                            break;
                    }
                }
                catch (Exception e) when (e is IOException || e is SocketException || e is ObjectDisposedException)
                {
                    // The window closed or reloaded.
                }
            }
        }

        private void Stream(NetworkStream stream)
        {
            Write(stream, MjpegHttp.StreamHeader());
            long sent = 0;
            while (!_stopped)
            {
                byte[]? frame;
                lock (_gate)
                {
                    while (_frameNumber == sent && !_stopped)
                    {
                        Monitor.Wait(_gate);
                    }

                    frame = _frame;
                    sent = _frameNumber;
                }

                if (frame == null)
                {
                    continue;
                }

                Write(stream, MjpegHttp.PartHeader(frame.Length));
                stream.Write(frame, 0, frame.Length);
                stream.Write(CrLf, 0, CrLf.Length);
                stream.Flush();
            }
        }

        private static readonly byte[] CrLf = { (byte)'\r', (byte)'\n' };

        private static void Write(NetworkStream stream, byte[] bytes) => stream.Write(bytes, 0, bytes.Length);
    }
}
