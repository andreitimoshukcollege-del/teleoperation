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

        /// <summary>The robot answered on this network: the address it answered from, or its mDNS A record.</summary>
        Lan,

        /// <summary>The robot answered only through Tailscale: the two are on different networks.</summary>
        Tailscale,

        /// <summary>Nothing answered: the configured address, as before discovery.</summary>
        Configured,

        /// <summary>Nothing answered and nothing was configured.</summary>
        NotFound,
    }

    /// <summary>What a query <see cref="RobotAddressResolver.NextQuery"/> asks for is.</summary>
    public enum RobotQueryKind
    {
        /// <summary>A <see cref="RobotDiscoveryCodec"/> query (nonce <see cref="RobotAddressResolver.Nonce"/>) to the robot's discovery port.</summary>
        Discovery,

        /// <summary>A <see cref="MdnsQueryCodec"/> A query for <see cref="RobotAddressResolver.MdnsName"/> (id <see cref="RobotAddressResolver.MdnsId"/>) to 224.0.0.251:5353, from every IPv4 interface (docs/adr/0015 §3).</summary>
        Mdns,
    }

    /// <summary>
    /// Finds the robot's address (docs/adr/0022-robot-discovery.md, decision 3). Pure: the caller owns
    /// the sockets, passes the time in seconds on any one clock, sends each query
    /// <see cref="NextQuery"/> asks for, hands every datagram that arrives to <see cref="OnDatagram"/>,
    /// and stops when <see cref="Source"/> is no longer <see cref="RobotAddressSource.Pending"/>.
    ///
    /// <para>Steps: ask through Tailscale; if the robot answers, ask each LAN address it reported, and
    /// the first that answers wins, else Tailscale. If Tailscale does not answer, ask for the robot's
    /// mDNS name (docs/adr/0015's legacy-unicast query) and ask the configured address; either answer
    /// wins. If nothing answers, the configured address.</para>
    /// </summary>
    public sealed class RobotAddressResolver
    {
        public const double ResendSeconds = 0.15;
        public const double TailscaleSeconds = 1.5;
        public const double LanProbeSeconds = 0.5;
        public const double LocalSearchSeconds = 1.0;

        private enum Step
        {
            AskTailscale,
            ProbeLan,
            LocalSearch,
            Done,
        }

        private readonly IPAddress? _tailscale;
        private readonly IPAddress? _configured;
        private readonly List<(IPAddress Address, RobotQueryKind Kind)> _targets = new List<(IPAddress, RobotQueryKind)>();
        private readonly uint[] _replyAddresses = new uint[RobotDiscoveryCodec.MaxAddresses];
        private readonly List<IPAddress> _reported = new List<IPAddress>();
        private readonly List<string> _log = new List<string>();
        private Step _step;
        private double _stepStart = double.NaN;
        private double _nextSend = double.NaN;
        private int _targetIndex;

        /// <param name="mdnsName">The robot's mDNS name (<c>jetrover-sinrg.local</c>); null or empty skips mDNS.</param>
        public RobotAddressResolver(IPAddress? tailscaleAddress, IPAddress? configuredAddress, string? mdnsName, uint nonce)
        {
            _tailscale = tailscaleAddress;
            _configured = configuredAddress;
            MdnsName = string.IsNullOrEmpty(mdnsName) ? null : mdnsName;
            Nonce = nonce;
            MdnsId = (ushort)((nonce & 0xFFFF) | 1);
            if (_tailscale != null)
            {
                Enter(Step.AskTailscale, double.NaN, new[] { (_tailscale, RobotQueryKind.Discovery) });
            }
            else
            {
                EnterLocalSearch(double.NaN);
            }
        }

        public RobotAddressSource Source { get; private set; } = RobotAddressSource.Pending;

        /// <summary>The settled address; null while pending, and for <see cref="RobotAddressSource.NotFound"/>.</summary>
        public IPAddress? Address { get; private set; }

        /// <summary>The robot's hostname, when it answered a discovery query.</summary>
        public string RobotName { get; private set; } = string.Empty;

        /// <summary>The LAN addresses the robot reported through Tailscale, if it did.</summary>
        public IReadOnlyList<IPAddress> ReportedLanAddresses => _reported;

        /// <summary>What happened, one line per step, for the log.</summary>
        public IReadOnlyList<string> Log => _log;

        public uint Nonce { get; }

        public string? MdnsName { get; }

        /// <summary>The mDNS query's id: never zero.</summary>
        public ushort MdnsId { get; }

        /// <summary>
        /// The next query to send now, if one is due. Call it until it returns false, every loop. Also
        /// applies the step timeouts.
        /// </summary>
        public bool NextQuery(double now, out IPAddress destination, out RobotQueryKind kind)
        {
            destination = IPAddress.None;
            kind = RobotQueryKind.Discovery;
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
            (destination, kind) = _targets[_targetIndex];
            _targetIndex++;
            if (_targetIndex == _targets.Count)
            {
                _targetIndex = 0;
                _nextSend = now + ResendSeconds;
            }

            return true;
        }

        /// <summary>A datagram that arrived from <paramref name="from"/>. Anything but an answer to this resolver's queries is ignored.</summary>
        public void OnDatagram(double now, IPAddress from, ReadOnlySpan<byte> datagram)
        {
            if (_step == Step.Done) return;
            from = Normalize(from);
            if (RobotDiscoveryCodec.TryDecodeReply(datagram, out uint nonce, _replyAddresses, out int count, out string name))
            {
                if (nonce == Nonce) OnDiscoveryReply(now, from, count, name);
                return;
            }

            if (_step == Step.LocalSearch && MdnsName != null &&
                MdnsQueryCodec.TryParseAnswer(datagram, MdnsId, MdnsName, out IPAddress address, out _))
            {
                address = Normalize(address);
                if (!IsLanCandidate(address))
                {
                    _log.Add($"{MdnsName} answered over mDNS with {address}, which no other device reaches it at; still asking");
                    return;
                }

                RobotName = MdnsName;
                Finish(RobotAddressSource.Lan, address, $"{MdnsName} answered over mDNS (from {from})");
            }
        }

        private void OnDiscoveryReply(double now, IPAddress from, int count, string name)
        {
            switch (_step)
            {
                case Step.AskTailscale:
                    if (!from.Equals(_tailscale)) return;
                    RobotName = name;
                    var lan = new List<(IPAddress, RobotQueryKind)>();
                    for (int i = 0; i < count; i++)
                    {
                        IPAddress a = RobotDiscoveryCodec.ToAddress(_replyAddresses[i]);
                        _reported.Add(a);
                        lan.Add((a, RobotQueryKind.Discovery));
                    }

                    _log.Add($"{name} answered through Tailscale ({_tailscale}); its LAN addresses: {(_reported.Count == 0 ? "none" : string.Join(", ", _reported))}");
                    if (lan.Count == 0) Finish(RobotAddressSource.Tailscale, _tailscale!, "it reported no LAN address");
                    else Enter(Step.ProbeLan, now, lan);
                    break;
                case Step.ProbeLan:
                    if (!_targets.Exists(t => t.Address.Equals(from))) return;
                    Finish(RobotAddressSource.Lan, from, "it answered on this network");
                    break;
                case Step.LocalSearch:
                    // A Tailscale answer would mean Tailscale works after all; this step is for the LAN.
                    if (IsTailscaleRange(from)) return;
                    RobotName = name;
                    Finish(RobotAddressSource.Lan, from, "it answered at the configured address");
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
                    string asked = string.Join(" or ", Asked());
                    _log.Add($"no answer from {(asked.Length == 0 ? "anything" : asked)} in {LocalSearchSeconds} s");
                    if (_configured != null) Finish(RobotAddressSource.Configured, _configured, "falling back to the configured address");
                    else Finish(RobotAddressSource.NotFound, null, "nothing configured to fall back to");
                    break;
            }
        }

        private IEnumerable<string> Asked()
        {
            foreach ((IPAddress address, RobotQueryKind kind) in _targets)
            {
                yield return kind == RobotQueryKind.Mdns ? $"mDNS ({MdnsName})" : address.ToString();
            }
        }

        private void EnterLocalSearch(double now)
        {
            var targets = new List<(IPAddress, RobotQueryKind)>();
            if (MdnsName != null) targets.Add((MdnsQueryCodec.Group, RobotQueryKind.Mdns));
            if (_configured != null && !_configured.Equals(_tailscale) && !IsTailscaleRange(_configured)) targets.Add((_configured, RobotQueryKind.Discovery));
            if (targets.Count == 0)
            {
                _step = Step.LocalSearch;
                Finish(_configured != null ? RobotAddressSource.Configured : RobotAddressSource.NotFound, _configured, "nothing to ask on this network");
                return;
            }

            Enter(Step.LocalSearch, now, targets);
        }

        private void Enter(Step step, double now, IReadOnlyList<(IPAddress Address, RobotQueryKind Kind)> targets)
        {
            _step = step;
            _stepStart = now;
            _nextSend = now;
            _targets.Clear();
            foreach ((IPAddress address, RobotQueryKind kind) in targets) _targets.Add((Normalize(address), kind));
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

        /// <summary>An IPv4 address another device on the LAN could reach the robot at: not loopback, link-local (169.254/16), Docker's 172.17/16 or Tailscale.</summary>
        public static bool IsLanCandidate(IPAddress address)
        {
            byte[] b = Normalize(address).GetAddressBytes();
            return b.Length == 4 && b[0] != 127 && !(b[0] == 169 && b[1] == 254) && !(b[0] == 172 && b[1] == 17) && !IsTailscaleRange(address);
        }
    }
}
