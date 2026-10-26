using System.Net.Sockets;
using System.Text;
using Teleop.CameraHost.Viewing;

namespace Teleop.CameraHost.Tests.Viewing
{
    /// <summary>
    /// <see cref="MjpegHttp"/> and <see cref="MjpegServer"/>: what the viewer window is sent, over a real
    /// loopback socket, and that a frame reaches it byte for byte.
    /// </summary>
    public class MjpegServerTests
    {
        [Theory]
        [InlineData("GET / HTTP/1.1", "/")]
        [InlineData("GET /stream?t=1 HTTP/1.1", "/stream")]
        [InlineData("POST / HTTP/1.1", null)]
        [InlineData("", null)]
        [InlineData(null, null)]
        public void RequestPath_takes_a_GET_path_without_its_query(string? line, string? expected)
        {
            Assert.Equal(expected, MjpegHttp.RequestPath(line));
        }

        [Fact]
        public void A_published_frame_reaches_the_stream_unchanged()
        {
            using var server = new MjpegServer(0, "test");
            var jpeg = new byte[5000];
            new Random(7).NextBytes(jpeg);

            using var client = new TcpClient("127.0.0.1", server.Port);
            NetworkStream stream = client.GetStream();
            stream.ReadTimeout = 5000;
            byte[] request = Encoding.ASCII.GetBytes("GET /stream HTTP/1.1\r\nHost: localhost\r\n\r\n");
            stream.Write(request, 0, request.Length);

            string head = ReadHeaders(stream);
            Assert.StartsWith("HTTP/1.1 200 OK", head);
            Assert.Contains($"multipart/x-mixed-replace; boundary={MjpegHttp.Boundary}", head);

            // The client may connect before or after the first frame; publish until one arrives.
            using var publishing = new CancellationTokenSource();
            var publisher = new Thread(() =>
            {
                while (!publishing.IsCancellationRequested)
                {
                    server.Publish((byte[])jpeg.Clone());
                    Thread.Sleep(20);
                }
            });
            publisher.Start();

            string part = ReadHeaders(stream);
            Assert.StartsWith($"--{MjpegHttp.Boundary}\r\nContent-Type: image/jpeg", part);
            Assert.Contains($"Content-Length: {jpeg.Length}", part);
            var received = new byte[jpeg.Length];
            int got = 0;
            while (got < received.Length)
            {
                int n = stream.Read(received, got, received.Length - got);
                Assert.True(n > 0, "the stream closed mid-frame");
                got += n;
            }

            publishing.Cancel();
            publisher.Join();
            Assert.Equal(jpeg, received);
        }

        [Fact]
        public void The_page_and_stats_are_served_and_anything_else_is_404()
        {
            using var server = new MjpegServer(0, "JetRover camera");
            server.SetStats("640x360 | 30 fps");

            Assert.Contains("<img src=\"/stream\"", Get(server.Port, "/"));
            Assert.EndsWith("640x360 | 30 fps", Get(server.Port, "/stats"));
            Assert.StartsWith("HTTP/1.1 404", Get(server.Port, "/nothing"));
        }

        private static string Get(int port, string path)
        {
            using var client = new TcpClient("127.0.0.1", port);
            NetworkStream stream = client.GetStream();
            stream.ReadTimeout = 5000;
            byte[] request = Encoding.ASCII.GetBytes($"GET {path} HTTP/1.1\r\nHost: localhost\r\n\r\n");
            stream.Write(request, 0, request.Length);
            using var reader = new StreamReader(stream, Encoding.UTF8);
            return reader.ReadToEnd();
        }

        /// <summary>Reads up to and including a blank line, byte by byte, so nothing after it is consumed.</summary>
        private static string ReadHeaders(NetworkStream stream)
        {
            var text = new StringBuilder();
            while (!text.ToString().EndsWith("\r\n\r\n"))
            {
                int b = stream.ReadByte();
                Assert.True(b >= 0, "the stream closed before its headers ended");
                text.Append((char)b);
            }

            return text.ToString();
        }
    }
}
