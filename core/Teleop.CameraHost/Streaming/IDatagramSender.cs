using System.Net;

namespace Teleop.CameraHost.Streaming
{
    /// <summary>
    /// Hands one datagram to the network. The seam that lets <see cref="CameraStreamer"/> be tested
    /// without a socket. Returns false, never throws, when the datagram could not be sent.
    /// </summary>
    internal interface IDatagramSender
    {
        bool TrySend(ReadOnlySpan<byte> datagram, EndPoint target);
    }
}
