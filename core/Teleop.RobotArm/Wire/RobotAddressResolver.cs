using System;
using System.Collections.Generic;
using System.Net;

namespace Teleop.RobotArm.Wire
{
    /// <summary>How <see cref="RobotAddressResolver"/> found the address it settled on.</summary>
    public enum RobotAddressSource
    {
        /// <summary>Still asking.</summary>
        Pending,

        /// <summary>The robot answered on this network: the address it answered from.</summary>
        Lan,

        /// <summary>The robot answered only through Tailscale: the two are on different networks.</summary>
        Tailscale,

        /// <summary>Nothing answered: the configured address, as before discovery.</summary>
        Configured,

        /// <summary>Nothing answered and nothing was configured.</summary>
        NotFound,
    }

    /// <summary>
    /// Finds the robot's address (docs/adr/0022-robot-discovery.md, decision 3). Pure: the caller owns
    /// the socket, passes the time in seconds on any one clock, sends each query
    /// <see cref="NextQuery"/> asks for, hands every datagram that arrives to <see cref="OnDatagram"/>,
    /// and stops when <see cref="Source"/> is no longer <see cref="RobotAddressSource.Pending"/>.
    ///
    /// <para>Steps: ask through Tailscale; if the robot answers, ask each LAN address it reported, and
    /// the first that answers wins, else Tailscale. If Tailscale does not answer, broadcast and ask the
    /// configured address; any answer wins. If nothing answers, the configured address.</para>
    /// </summary>
    public sealed class RobotAddressResolver
    {
        public const double ResendSeconds = 0.15;
        public const double TailscaleSeconds = 1.5;
        public const double LanProbeSeconds = 0.5;
        public const double LocalSearchSeconds = 0.6;

        private enum Step
        {
            AskTailscale,
            ProbeLan,
            LocalSearch,
            Done,
        }

        private readonly IPAddress? _tailscale;
        private readonly IPAddress? _configured;
        private readonly uint _nonce;
        private readonly List<IPAddress> _targets = new List<IPAddress>();
        private readonly uint[] _replyAddresses = new uint[RobotDiscoveryCodec.MaxAddresses];
        private readonly List<IPAddress> _reported = new List<IPAddress>();
        private readonly List<string> _log = new List<string>();
        private Step _step;
        private double _stepStart = double.NaN;
        private double _nextSend = double.NaN;
        private int _targetIndex;

        public RobotAddressResolver(IPAddress? tailscaleAddress, IPAddress? configuredAddress, uint nonce)
        {
            _tailscale = tailscaleAddress;
            _configured = configuredAddress;
            _nonce = nonce;
            if (_tailscale != null)
            {
                Enter(Step.AskTailscale, double.NaN, new[] { _tailscale });
            }
            else
            {
                EnterLocalSearch(double.NaN);
            }
        }

        public RobotAddressSource Source { get; private set; } = RobotAddressSource.Pending;

        /// <summary>The settled address; null while pending, and for <see cref="RobotAddressSource.NotFound"/>.</summary>
        public IPAddress? Address { get; private set; }

        /// <summary>The robot's hostname, when it answered.</summary>
        public string RobotName { get; private set; } = string.Empty;

        /// <summary>The LAN addresses the robot reported through Tailscale, if it did.</summary>
        public IReadOnlyList<IPAddress> ReportedLanAddresses => _reported;

        /// <summary>What happened, one line per step, for the log.</summary>
        public IReadOnlyList<string> Log => _log;

        public uint Nonce => _nonce;

        /// <summary>
        /// The next query to send now, if one is due: to <paramref name="destination"/>, which is
        /// <see cref="IPAddress.Broadcast"/> for the broadcast. Call it until it returns false, every loop.
        /// Also applies the step timeouts.
        /// </summary>
        public bool NextQuery(double now, out IPAddress destination)
        {
            destination = IPAddress.None;
            if (_step == Step.Done) return false;
            if (double.IsNaN(_stepStart))
            {
                _stepStart = now;
                _nextSend = now;
            }

            if (now - _stepStart >= StepSeconds(_step))
            {
                Timeout(now);
                if (_step == Step.Done) return false;
            }

            if (now < _nextSend) return false;
            destination = _targets[_targetIndex];
            _targetIndex++;
            if (_targetIndex == _targets.Count)
            {
                _targetIndex = 0;
                _nextSend = now + ResendSeconds;
            }

            return true;
        }

        /// <summary>A datagram that arrived from <paramref name="from"/>. Anything but a reply to this resolver's query is ignored.</summary>
        public void OnDatagram(double now, IPAddress from, ReadOnlySpan<byte> datagram)
        {
            if (_step == Step.Done) return;
            if (!RobotDiscoveryCodec.TryDecodeReply(datagram, out uint nonce, _replyAddresses, out int count, out string name) || nonce != _nonce) return;
            from = Normalize(from);
            switch (_step)
            {
                case Step.AskTailscale:
                    if (!from.Equals(_tailscale)) return;
                    RobotName = name;
                    var lan = new List<IPAddress>();
                    for (int i = 0; i < count; i++) lan.Add(RobotDiscoveryCodec.ToAddress(_replyAddresses[i]));
                    _reported.AddRange(lan);
                    _log.Add($"{name} answered through Tailscale ({_tailscale}); its LAN addresses: {(lan.Count == 0 ? "none" : string.Join(", ", lan))}");
                    if (lan.Count == 0)
                    {
                        Finish(RobotAddressSource.Tailscale, _tailscale!, "it reported no LAN address");
                    }
                    else
                    {
                        Enter(Step.ProbeLan, now, lan);
                    }

                    break;
                case Step.ProbeLan:
                    if (!_targets.Contains(from)) return;
                    Finish(RobotAddressSource.Lan, from, "it answered on this network");
                    break;
                case Step.LocalSearch:
                    // A Tailscale answer would mean Tailscale works after all; this step is for the LAN.
                    if (IsTailscaleRange(from)) return;
                    RobotName = name;
                    Finish(RobotAddressSource.Lan, from, "it answered on this network");
                    break;
            }
        }

        private void Timeout(double now)
        {
            switch (_step)
            {
                case Step.AskTailscale:
                    _log.Add($"no answer through Tailscale ({_tailscale}) in {TailscaleSeconds} s: Tailscale is not running here, or the robot is offline");
                    EnterLocalSearch(now);
                    break;
                case Step.ProbeLan:
                    Finish(RobotAddressSource.Tailscale, _tailscale!, $"none of its LAN addresses answered in {LanProbeSeconds} s, so this device is on another network");
                    break;
                case Step.LocalSearch:
                    _log.Add($"no answer to the broadcast{(_configured != null ? $" or from {_configured}" : string.Empty)} in {LocalSearchSeconds} s");
                    if (_configured != null) Finish(RobotAddressSource.Configured, _configured, "falling back to the configured address");
                    else Finish(RobotAddressSource.NotFound, null, "nothing configured to fall back to");
                    break;
            }
        }

        private void EnterLocalSearch(double now)
        {
            var targets = new List<IPAddress> { IPAddress.Broadcast };
            if (_configured != null && !_configured.Equals(_tailscale) && !IsTailscaleRange(_configured)) targets.Add(_configured);
            Enter(Step.LocalSearch, now, targets);
        }

        private void Enter(Step step, double now, IReadOnlyList<IPAddress> targets)
        {
            _step = step;
            _stepStart = now;
            _nextSend = now;
            _targets.Clear();
            foreach (IPAddress t in targets) _targets.Add(Normalize(t));
            _targetIndex = 0;
        }

        private void Finish(RobotAddressSource source, IPAddress? address, string why)
        {
            Source = source;
            Address = address;
            _step = Step.Done;
            _log.Add($"using {address?.ToString() ?? "nothing"} ({source}): {why}");
        }

        private static double StepSeconds(Step step) => step switch
        {
            Step.AskTailscale => TailscaleSeconds,
            Step.ProbeLan => LanProbeSeconds,
            Step.LocalSearch => LocalSearchSeconds,
            _ => double.PositiveInfinity,
        };

        private static IPAddress Normalize(IPAddress address) => address.IsIPv4MappedToIPv6 ? address.MapToIPv4() : address;

        /// <summary>Tailscale's addresses: 100.64.0.0/10.</summary>
        public static bool IsTailscaleRange(IPAddress address)
        {
            byte[] b = Normalize(address).GetAddressBytes();
            return b.Length == 4 && b[0] == 100 && (b[1] & 0xC0) == 64;
        }
    }
}
