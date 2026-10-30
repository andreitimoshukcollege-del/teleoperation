using System;
using System.IO;
using System.Text.Json;
using Teleop.RobotArm.Types;

namespace Teleop.RobotHost.Tests.Backend
{
    /// <summary>The committed FR3 Duo profile (core/RobotProfiles/fr3-duo.json), loaded once for the serial-chain host's tests.</summary>
    internal static class SerialChainTestProfile
    {
        public static readonly string Path = FindProfile();

        public static SerialChainProfile Fr3Duo() => Teleop.RobotHost.Plant.SerialChainProfileJson.Load(Path);

        private static string FindProfile()
        {
            for (DirectoryInfo? dir = new DirectoryInfo(AppContext.BaseDirectory); dir != null; dir = dir.Parent)
            {
                string candidate = System.IO.Path.Combine(dir.FullName, "core", "RobotProfiles", "fr3-duo.json");
                if (File.Exists(candidate)) return candidate;
            }

            throw new InvalidOperationException($"no core/RobotProfiles/fr3-duo.json above {AppContext.BaseDirectory}");
        }
    }
}
