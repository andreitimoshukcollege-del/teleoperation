using System;
using Teleop.RobotArm.World;
using Teleop.RobotHost.ArmServer;

namespace Teleop.RobotHost.World
{
    /// <summary>
    /// What simulates the world's objects behind RobotHost (docs/adr/0031 amendment 2): the Isaac bridge over the arm-server
    /// wire (<see cref="ArmServerClient"/>), or <see cref="StaticWorld"/> beside the simulated arm. RobotHost decides where
    /// objects go (<see cref="WorldHost"/>); the backend applies or refuses each placement and reports where they are.
    /// Brought up to date by the arm backend's <c>Poll</c> (or the static world's own).
    /// </summary>
    public interface IWorldBackend
    {
        /// <summary>Whether the backend has said which scene it runs; nothing is placed before.</summary>
        bool HasWorld { get; }

        /// <summary>The scene file's sha256 the backend runs; empty before <see cref="HasWorld"/>.</summary>
        ReadOnlySpan<byte> SceneSha256 { get; }

        /// <summary>True for physics (Isaac), false for poses that only ever are where they were placed.</summary>
        bool Physics { get; }

        void Place(uint request, PlaceCause cause, in ObjectPose pose, long hostTicks);

        /// <summary>The newest complete snapshot since the last call: every object's pose, sensed on the host's clock.</summary>
        bool TryTakeSnapshot(Span<ObjectPose> poses, out int count, out long sensedTicks, out uint revision);

        /// <summary>The next placement's result, in the order they came.</summary>
        bool TryTakePlaced(out uint request, out PlaceOutcome outcome, out ObjectPose pose);
    }
}
