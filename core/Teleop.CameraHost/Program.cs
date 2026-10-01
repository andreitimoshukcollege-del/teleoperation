namespace Teleop.CameraHost
{
    /// <summary>
    /// The robot-side camera sender of docs/adr/0014-camera-frame-downlink.md. Today it has one verb,
    /// <c>timestamp-spike</c>, the ADR's first implementation step; the streaming sender follows once
    /// that passes on the Jetson.
    /// </summary>
    internal static class Program
    {
        private const string Usage =
            "Usage: Teleop.CameraHost timestamp-spike [--device /dev/video0] [--width 640] [--height 480] " +
            "[--fps 30] [--frames 150]\n" +
            "  Captures native MJPEG frames and checks that the driver's capture timestamps are\n" +
            "  CLOCK_MONOTONIC and agree with Stopwatch. Linux only. Exit 0 pass, 1 fail, 2 setup error.";

        private static int Main(string[] args)
        {
            if (args.Length == 0 || args[0] != "timestamp-spike")
            {
                Console.Error.WriteLine(Usage);
                return 2;
            }

            if (!OperatingSystem.IsLinux())
            {
                Console.Error.WriteLine("error: V4L2 capture needs Linux; run this on the Jetson.");
                return 2;
            }

            string device = "/dev/video0";
            uint width = 640, height = 480, fps = 30;
            int frames = 150;
            for (int i = 1; i < args.Length; i++)
            {
                bool hasValue = i + 1 < args.Length;
                switch (args[i])
                {
                    case "--device" when hasValue: device = args[++i]; break;
                    case "--width" when hasValue && uint.TryParse(args[i + 1], out width): i++; break;
                    case "--height" when hasValue && uint.TryParse(args[i + 1], out height): i++; break;
                    case "--fps" when hasValue && uint.TryParse(args[i + 1], out fps): i++; break;
                    case "--frames" when hasValue && int.TryParse(args[i + 1], out frames): i++; break;
                    default:
                        Console.Error.WriteLine($"error: unrecognised or invalid argument '{args[i]}'");
                        Console.Error.WriteLine(Usage);
                        return 2;
                }
            }

            if (width == 0 || height == 0 || fps == 0 || frames <= WarmUp)
            {
                Console.Error.WriteLine($"error: width, height and fps must be positive and frames more than {WarmUp}.");
                return 2;
            }

            return TimestampSpike.Run(device, width, height, fps, frames);
        }

        private const int WarmUp = 5;
    }
}
