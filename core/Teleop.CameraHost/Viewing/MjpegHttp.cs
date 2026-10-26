using System.Text;

namespace Teleop.CameraHost.Viewing
{
    /// <summary>
    /// The bytes of the viewer's tiny HTTP server (<see cref="MjpegServer"/>): a browser shows a
    /// <c>multipart/x-mixed-replace</c> response of JPEGs in an <c>&lt;img&gt;</c> as live video, so the
    /// camera's own frames go to the window unchanged, with no decoder and no GUI library here.
    /// </summary>
    internal static class MjpegHttp
    {
        public const string Boundary = "teleopframe";

        /// <summary>The path of an HTTP/1.x request line (<c>GET /stream HTTP/1.1</c>), without its query; null when it is not a GET.</summary>
        public static string? RequestPath(string? requestLine)
        {
            if (requestLine == null)
            {
                return null;
            }

            string[] parts = requestLine.Split(' ');
            if (parts.Length < 2 || parts[0] != "GET")
            {
                return null;
            }

            int query = parts[1].IndexOf('?');
            return query < 0 ? parts[1] : parts[1].Substring(0, query);
        }

        public static byte[] StreamHeader() => Encoding.ASCII.GetBytes(
            "HTTP/1.1 200 OK\r\n" +
            $"Content-Type: multipart/x-mixed-replace; boundary={Boundary}\r\n" +
            "Cache-Control: no-store\r\n" +
            "Connection: close\r\n\r\n");

        /// <summary>What goes before one JPEG in the stream; the JPEG is followed by <c>\r\n</c>.</summary>
        public static byte[] PartHeader(int jpegBytes) => Encoding.ASCII.GetBytes(
            $"--{Boundary}\r\nContent-Type: image/jpeg\r\nContent-Length: {jpegBytes}\r\n\r\n");

        public static byte[] Response(string status, string contentType, string body)
        {
            byte[] content = Encoding.UTF8.GetBytes(body);
            byte[] head = Encoding.ASCII.GetBytes(
                $"HTTP/1.1 {status}\r\nContent-Type: {contentType}\r\nContent-Length: {content.Length}\r\n" +
                "Cache-Control: no-store\r\nConnection: close\r\n\r\n");
            var all = new byte[head.Length + content.Length];
            head.CopyTo(all, 0);
            content.CopyTo(all, head.Length);
            return all;
        }

        /// <summary>The page: the stream filling the window, and a status line from <c>/stats</c> once a second.</summary>
        public static string Page(string title) => @"<!doctype html>
<html><head><meta charset=""utf-8""><title>" + title + @"</title>
<style>
html,body{margin:0;height:100%;background:#000;color:#ddd;font:13px system-ui,sans-serif;overflow:hidden}
img{position:absolute;inset:0;width:100%;height:100%;object-fit:contain}
#s{position:absolute;left:8px;bottom:6px;background:rgba(0,0,0,.6);padding:2px 6px;border-radius:3px}
</style></head><body>
<img src=""/stream"" alt="""">
<div id=""s"">waiting for the camera...</div>
<script>
const s=document.getElementById('s');
setInterval(async()=>{try{const r=await fetch('/stats');s.textContent=await r.text();}catch(e){s.textContent='viewer stopped';}},1000);
</script>
</body></html>";
    }
}
