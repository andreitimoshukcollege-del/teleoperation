using System;
using System.Diagnostics;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using Teleop.RobotArm.Wire;

namespace Teleop.Eval.ArmPower
{
    /// <summary>
    /// <c>arm-power --request relax|hold</c>: the headset's ROBOT OFF and ROBOT ON from the PC
    /// (<c>just robot-off</c>, <c>just robot-on</c>; docs/adr/0023 decision 5). Sends the operator
    /// message (docs/adr/0018 §9) as a tool, which never opens an automatic recording, repeating the
    /// request every 0.3 s until RobotHost's status answers it:
    /// <list type="bullet">
    /// <item><b>relax</b>: RobotHost parks the arm at its rest pose (its own collision-checked path, at
    /// its park speed), waits until the readings say it is there, then switches torque off. Exits 0 once
    /// relaxed; 1 if RobotHost refuses (the reason is printed) or nothing answers in time.</item>
    /// <item><b>hold</b>: torque back on where the arm is; nothing moves. Exits 0 once holding.</item>
    /// </list>
    /// A headset that is on and commanding asks RobotHost to hold whenever it sees the arm parked, so
    /// turn it off there first; a hold request from elsewhere is reported.
    /// </summary>
    public static class ArmPowerCommand
    {
        public static int Run(string[] args)
        {
            string? host = null, requestName = null;
            int port = 6002;
            double timeoutSeconds = 30;
            for (int i = 1; i < args.Length; i++)
            {
                string Next() => i + 1 < args.Length ? args[++i] : throw new ArgumentException($"{args[i]} needs a value");
                switch (args[i])
                {
                    case "--remote-host": host = Next(); break;
                    case "--remote-port": port = int.Parse(Next(), CultureInfo.InvariantCulture); break;
                    case "--request": requestName = Next(); break;
                    case "--timeout-seconds": timeoutSeconds = double.Parse(Next(), CultureInfo.InvariantCulture); break;
                    default: return Usage();
                }
            }

            if (host == null || !IPAddress.TryParse(host, out IPAddress? address) || (requestName != "relax" && requestName != "hold"))
            {
                return Usage();
            }

            return Request(new IPEndPoint(address, port), requestName == "relax" ? ArmPowerRequest.Relax : ArmPowerRequest.Hold,
                timeoutSeconds, out _);
        }

        /// <summary>
        /// Sends <paramref name="request"/> until RobotHost's status answers it (relaxed for a relax, holding
        /// for a hold), printing the progress. 0 when it did; 1 on a refusal or a timeout.
        /// <paramref name="answered"/> says whether any status came back at all (false: RobotHost is older
        /// than park and relax, or not there).
        /// </summary>
        public static int Request(IPEndPoint robot, ArmPowerRequest request, double timeoutSeconds, out bool answered)
        {
            bool relax = request == ArmPowerRequest.Relax;
            OperatorMotionMode mode = relax ? OperatorMotionMode.Off : OperatorMotionMode.On;
            using var socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            socket.Bind(new IPEndPoint(IPAddress.Any, 0));
            socket.ReceiveTimeout = 100;

            var buffer = new byte[256];
            long frequency = Stopwatch.Frequency, start = Stopwatch.GetTimestamp(), nextSend = start;
            long end = start + (long)(timeoutSeconds * frequency);
            uint sequence = 0, firstSequence = 0;
            ArmPowerState? lastState = null;
            int lastRemaining = int.MinValue;
            answered = false;
            Console.WriteLine(relax
                ? $"arm-power: asking RobotHost at {robot} to park the arm at its rest pose and switch torque off"
                : $"arm-power: asking RobotHost at {robot} to switch torque on (the arm holds where it is)");

            while (Stopwatch.GetTimestamp() < end)
            {
                long now = Stopwatch.GetTimestamp();
                if (now >= nextSend)
                {
                    ArmPowerCodec.TryEncodeRequest(OperatorClientKind.Tool, mode, request, ++sequence, now, buffer, out int n);
                    if (firstSequence == 0) firstSequence = sequence;
                    socket.SendTo(buffer, 0, n, SocketFlags.None, robot);
                    nextSend = now + frequency * 3 / 10;
                }

                int received;
                try
                {
                    received = socket.Receive(buffer);
                }
                catch (SocketException)
                {
                    continue;
                }

                if (!ArmPowerCodec.TryDecodeStatus(buffer.AsSpan(0, received), out ArmPowerState state, out ArmPowerReason reason,
                        out uint echo, out _, out _, out float remainingDeg))
                {
                    continue;
                }

                // Only a status that has seen one of our requests answers it.
                if (echo < firstSequence) continue;
                answered = true;
                int remaining = float.IsNaN(remainingDeg) ? -1 : (int)Math.Round(remainingDeg);
                if (state != lastState || (state == ArmPowerState.Parking && remaining != lastRemaining))
                {
                    Console.WriteLine(state switch
                    {
                        ArmPowerState.Parking => $"  parking: {(remaining >= 0 ? remaining.ToString(CultureInfo.InvariantCulture) + " deg" : "?")} to the rest pose",
                        ArmPowerState.Relaxed => "  relaxed: at the rest pose, torque off",
                        _ => reason == ArmPowerReason.None ? "  holding: torque on" : $"  holding: {ArmPowerCodec.Describe(reason)}",
                    });
                    lastState = state;
                    lastRemaining = remaining;
                }

                if (relax && state == ArmPowerState.Relaxed)
                {
                    return 0;
                }

                if (relax && state == ArmPowerState.Holding && reason != ArmPowerReason.None)
                {
                    if (reason == ArmPowerReason.HoldRequested)
                        Console.Error.WriteLine("arm-power: something else asked RobotHost to hold (a headset that is on?): turn it off there and run this again");
                    else
                        Console.Error.WriteLine($"arm-power: RobotHost refused: {ArmPowerCodec.Describe(reason)}");
                    return 1;
                }

                if (!relax && state == ArmPowerState.Holding)
                {
                    return 0;
                }
            }

            Console.Error.WriteLine(answered
                ? $"arm-power: no {(relax ? "relaxed" : "holding")} state within {timeoutSeconds:0} s (last: {lastState})"
                : $"arm-power: no status from {robot} in {timeoutSeconds:0} s: is RobotHost running, and new enough (docs/adr/0023 decision 5)?");
            return 1;
        }

        private static int Usage()
        {
            Console.Error.WriteLine("usage: arm-power --remote-host <ip> --request relax|hold [--remote-port 6002] [--timeout-seconds 30]");
            return 64;
        }
    }
}
