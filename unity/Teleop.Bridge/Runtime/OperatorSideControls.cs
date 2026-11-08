using System;
using Teleop.Core.Contracts;
using Teleop.RobotArm.Operator;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotArm.World;
using UnityEngine;
using CoreVec = System.Numerics.Vector3;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.Bridge
{
    /// <summary>
    /// What any robot's operator bridge offers beside driving the arm (docs/adr/0034 decision 7): the world (NEW LAYOUT, the twin's
    /// objects, moves), pointing at objects to drag them (docs/adr/0031 amendment 5, Core's <see cref="PointerDrag"/>), and the
    /// RECORD button's scenario recording (docs/adr/0033). Plain C#, over Core's <see cref="JointPortSideChannels"/>: the FR3's and
    /// the JetRover's bridges each make one, and the scene's twin, buttons and pointer reach it through
    /// <see cref="OperatorBridgeBase.SideControls"/>, whichever robot it is.
    /// </summary>
    public sealed class OperatorSideControls
    {
        private readonly JointPortSideChannels _side;
        private readonly ITimeAuthority _clock;
        private readonly Func<Transform> _root;
        private readonly Func<bool> _motionOn;
        private readonly PointerDrag _pointer;
        private bool _hasPendingMove;
        private ObjectPose _pendingMove;
        private int _selectedScenario;

        /// <param name="root">The robot's base frame in the scene: the twin's root, where the world's poses are given.</param>
        /// <param name="motionOn">Whether the robot is on (the hands drive the arms): world edits and pointing wait for it to be off.</param>
        public OperatorSideControls(JointPortSideChannels side, ITimeAuthority clock, Func<Transform> root, Func<bool> motionOn)
        {
            _side = side ?? throw new ArgumentNullException(nameof(side));
            _clock = clock ?? throw new ArgumentNullException(nameof(clock));
            _root = root ?? throw new ArgumentNullException(nameof(root));
            _motionOn = motionOn ?? throw new ArgumentNullException(nameof(motionOn));
            _pointer = side.World != null ? new PointerDrag(side.World.Scene) : null;
        }

        /// <summary>Another way of dragging objects, the FR3's EDIT OBJECTS curl: while it is on, pointing drags nothing, and the twin shows its ghost.</summary>
        public Func<bool> OtherDragOn { get; set; } = () => false;
        public Func<(bool Dragging, byte Id, RigidTransform Preview)> OtherDrag { get; set; } = () => (false, 0, RigidTransform.Identity);

        // --- the world ------------------------------------------------------------------------------------------------

        public WorldState World => _side.World;
        public bool HasWorld => _side.World != null;
        public bool WorldSceneMismatch => _side.World != null && _side.World.SceneMismatch;
        private bool WorldRunning => _side.World != null && _side.World.HasStatus && _side.World.Status.Running && !_side.World.SceneMismatch;

        /// <summary>Whether NEW LAYOUT can be asked for now: a world is running, motion is off, and no edit is waiting.</summary>
        public bool CanRequestNewLayout => WorldRunning && !_motionOn() && !_side.WorldEditPending;

        public bool RequestNewLayout()
        {
            if (!CanRequestNewLayout) return false;
            bool sent = _side.RequestWorldEdit(WorldEditOp.NewLayout, default, _clock.NowTicks);
            if (sent) Debug.Log("[world] NEW LAYOUT requested");
            return sent;
        }

        /// <summary>A move to ask RobotHost for, once nothing else waits (the link re-sends it until echoed).</summary>
        public void QueueMove(in ObjectPose pose)
        {
            _pendingMove = pose;
            _hasPendingMove = true;
        }

        public bool EditPending => _hasPendingMove || _side.WorldEditPending;

        /// <summary>RobotHost's answer to the last edit, in words; empty before any.</summary>
        public string LastEditOutcome
        {
            get
            {
                WorldState w = _side.World;
                if (w == null || !w.HasStatus || w.Status.EditSequence == 0) return "";
                switch (w.Status.EditOutcome)
                {
                    case 0: return "done";
                    case 1: return "refused: an arm is in the way";
                    case 2: return "refused: another object is there";
                    case 3: return "refused: off the table";
                    case 5: return "refused: that one is fixed";
                    case 6: return "refused: an arm is touching it";
                    case 7: return "refused: the robot is on";
                    case 9: return "no answer from the world";
                    default: return "refused";
                }
            }
        }

        /// <summary>Whether an object is being dragged (by a pointer, or the other way), and where it would go: the twin's ghost.</summary>
        public bool EditDragging => PointerDragging || OtherDrag().Dragging;
        public byte EditDraggedId => PointerDragging ? _pointer.DraggedId : OtherDrag().Id;
        public RigidTransform EditPreview => PointerDragging ? _pointer.Preview : OtherDrag().Preview;

        // --- pointing --------------------------------------------------------------------------------------------------

        /// <summary>Whether a pointing ray may drag objects now: a running world with this headset's scene file, and the robot off.</summary>
        public bool CanPointerDrag => _pointer != null && WorldRunning && !_motionOn() && !OtherDragOn();

        public bool PointerDragging => _pointer != null && _pointer.IsDragging;

        /// <summary>How far along a world-space ray the first object it hits is (m); negative when none, or pointing cannot drag now.</summary>
        public float PointerHit(Vector3 origin, Vector3 direction)
        {
            if (!CanPointerDrag) return -1f;
            RootRay(origin, direction, out CoreVec o, out CoreVec d);
            return _pointer.Hit(o.X, o.Y, o.Z, d.X, d.Y, d.Z, _side.World, out double distance) >= 0 ? (float)distance : -1f;
        }

        /// <summary>One hand's ray this frame and whether it pinches; letting go of a dragged object asks RobotHost to put it there.</summary>
        public void UpdatePointer(int hand, bool tracked, bool pinched, Vector3 origin, Vector3 direction)
        {
            if (_pointer == null) return;
            bool can = CanPointerDrag;
            RootRay(origin, direction, out CoreVec o, out CoreVec d);
            bool was = _pointer.IsDragging;
            _pointer.Update(hand, tracked && can, pinched && can, o.X, o.Y, o.Z, d.X, d.Y, d.Z, _side.World);
            if (!was && _pointer.IsDragging) Debug.Log($"[world] pointer grabbed object {_pointer.DraggedId}");
            if (_pointer.TryTakeRequest(out ObjectPose move)) QueueMove(move);
        }

        /// <summary>Each frame, after the link's tick: sends a queued move once nothing else waits.</summary>
        public void Update(long now)
        {
            if (_hasPendingMove && !_side.WorldEditPending && _side.RequestWorldEdit(WorldEditOp.Move, _pendingMove, now))
            {
                _hasPendingMove = false;
                Debug.Log($"[world] move {_pendingMove.Id} to ({_pendingMove.X:0.000}, {_pendingMove.Y:0.000}) requested");
            }
        }

        private void RootRay(Vector3 origin, Vector3 direction, out CoreVec o, out CoreVec d)
        {
            Transform root = _root();
            if (root == null)
            {
                o = origin.ToCore();
                d = CoreVec.Normalize(direction.ToCore());
                return;
            }

            o = root.InverseTransformPoint(origin).ToCore();
            CoreVec far = root.InverseTransformPoint(origin + direction).ToCore();
            d = CoreVec.Normalize(far - o);
        }

        // --- scenario recording ----------------------------------------------------------------------------------------

        public RecordingState Recording => _side.Recording;

        /// <summary>Whether RobotHost records scenarios (it has answered with a status).</summary>
        public bool RecordingAvailable => _side.Recording.HasStatus && _side.Recording.Status.State != RecordState.Unavailable;

        public bool IsRecording => RecordingAvailable && _side.Recording.Status.State == RecordState.Recording;
        public bool RecordPending => _side.RecordRequestPending;
        public int ScenarioCount => _side.Recording.Count;

        public int SelectedScenario
        {
            get
            {
                int n = _side.Recording.Count;
                return n == 0 ? 0 : Math.Max(0, Math.Min(n - 1, _selectedScenario));
            }
        }

        public string SelectedScenarioName => _side.Recording.Name(SelectedScenario);
        public string TakeScenarioName => _side.Recording.Name(_side.Recording.Status.Scenario);
        public int TakeNumber => _side.Recording.Status.Take;
        public int TakeSeconds => (int)(_side.Recording.Status.ElapsedMs / 1000);
        public int RecordingFreeGigabytes => _side.Recording.Status.FreeGigabytes;

        public void StepScenario(int step)
        {
            int n = _side.Recording.Count;
            if (n == 0 || IsRecording) return;
            _selectedScenario = ((SelectedScenario + step) % n + n) % n;
        }

        public bool ToggleRecording()
        {
            if (!RecordingAvailable || RecordPending) return false;
            bool stop = IsRecording;
            bool sent = _side.RequestRecord(stop ? RecordOp.Stop : RecordOp.Start, SelectedScenario, _clock.NowTicks);
            if (sent) Debug.Log(stop ? "[record] STOP asked" : $"[record] RECORD '{SelectedScenarioName}' asked");
            return sent;
        }

        public bool DiscardLastTake()
        {
            if (!RecordingAvailable || RecordPending || IsRecording) return false;
            bool sent = _side.RequestRecord(RecordOp.DiscardLast, SelectedScenario, _clock.NowTicks);
            if (sent) Debug.Log("[record] DISCARD LAST asked");
            return sent;
        }

        public string LastRecordOutcome
        {
            get
            {
                if (!_side.Recording.HasStatus || _side.Recording.Status.Sequence == 0) return "";
                switch (_side.Recording.Status.Outcome)
                {
                    case RecordOutcome.Started: return "recording";
                    case RecordOutcome.Saved: return "take saved";
                    case RecordOutcome.Discarded: return "last take set aside";
                    case RecordOutcome.Busy: return "busy: try again";
                    case RecordOutcome.UnknownScenario: return "the list changed: pick again";
                    case RecordOutcome.LowDisk: return "the disk is nearly full";
                    case RecordOutcome.Failed: return "could not record: see RobotHost's log";
                    case RecordOutcome.NothingToDiscard: return "no take to set aside";
                    case RecordOutcome.StoppedByItself: return "the take stopped by itself";
                    default: return "";
                }
            }
        }
    }
}
