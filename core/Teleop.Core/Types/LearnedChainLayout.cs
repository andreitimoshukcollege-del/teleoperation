using System;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Types
{
    /// <summary>
    /// What layout v4 needs to know about a serial-chain robot (docs/adr/0028 decision 1), restated from its profile so
    /// Core needs no reference to it (<c>Teleop.RobotArm</c>'s <c>SerialChainShaping.LearnedLayout</c> builds it):
    /// <list type="bullet">
    /// <item>the window's rows in order, each a motor;</item>
    /// <item>the arm each row belongs to;</item>
    /// <item>the row's identity within its arm: joints 0 to <see cref="ArmJoints"/> - 1, then <see cref="ArmJoints"/>
    /// for the hand;</item>
    /// <item>gravity's unit direction in each arm's mounted base frame.</item>
    /// </list>
    /// </summary>
    public sealed class LearnedChainLayout
    {
        /// <summary>Layout v4's arms have seven joints; a hand's identity is this index.</summary>
        public const int ArmJoints = 7;

        private readonly byte[] _motors;
        private readonly int[] _arm;
        private readonly int[] _identity;
        private readonly double[] _gravity;

        public LearnedChainLayout(byte[] motors, int[] armOfRow, int[] identity, double[] gravityPerArm)
        {
            if (motors == null || armOfRow == null || identity == null || gravityPerArm == null) throw new ArgumentNullException(nameof(motors));
            if (motors.Length == 0 || armOfRow.Length != motors.Length || identity.Length != motors.Length) throw new ArgumentException("one arm and one identity per row");
            int arms = 0;
            for (int r = 0; r < motors.Length; r++)
            {
                if (armOfRow[r] < 0 || identity[r] < 0 || identity[r] > ArmJoints) throw new ArgumentException($"row {r}: arm {armOfRow[r]}, identity {identity[r]}");
                if (armOfRow[r] + 1 > arms) arms = armOfRow[r] + 1;
                for (int k = 0; k < r; k++) if (motors[k] == motors[r]) throw new ArgumentException($"motor {motors[r]} is two rows");
            }

            if (gravityPerArm.Length != 3 * arms) throw new ArgumentException($"gravity needs 3 values for each of {arms} arms");
            for (int a = 0; a < arms; a++)
            {
                int joints = 0;
                for (int r = 0; r < motors.Length; r++) if (armOfRow[r] == a && identity[r] < ArmJoints) joints++;
                if (joints != ArmJoints) throw new ArgumentException($"arm {a} has {joints} joints; layout v4 is for {ArmJoints}-joint arms");
            }

            _motors = (byte[])motors.Clone();
            _arm = (int[])armOfRow.Clone();
            _identity = (int[])identity.Clone();
            _gravity = (double[])gravityPerArm.Clone();
            Arms = arms;
        }

        public int Rows => _motors.Length;
        public int Arms { get; }

        public byte MotorOf(int row) => _motors[row];
        public int ArmOf(int row) => _arm[row];
        public int IdentityOf(int row) => _identity[row];
        public bool IsHand(int row) => _identity[row] == ArmJoints;

        /// <summary>Component <paramref name="axis"/> (0-2) of gravity's direction in arm <paramref name="arm"/>'s base.</summary>
        public double Gravity(int arm, int axis) => _gravity[3 * arm + axis];

        public int RowOf(byte motor)
        {
            for (int r = 0; r < _motors.Length; r++) if (_motors[r] == motor) return r;
            return -1;
        }
    }
}
