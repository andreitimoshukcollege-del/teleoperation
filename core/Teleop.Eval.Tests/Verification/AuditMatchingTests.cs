using Teleop.Eval.Verification;
using Xunit;

namespace Teleop.Eval.Tests.Verification
{
    /// <summary>The registry and impairment checks' type matching: whole identifiers, comments excluded.</summary>
    public sealed class AuditMatchingTests
    {
        [Fact]
        public void A_name_inside_a_longer_name_does_not_count()
        {
            Assert.False(AuditCommand.NamesType("new KalmanPosePredictor(cfg)", "Kalman"));
            Assert.False(AuditCommand.NamesType("new MyKalman()", "Kalman"));
            Assert.True(AuditCommand.NamesType("[\"kalman\"] = c => new Kalman(c),", "Kalman"));
        }

        [Fact]
        public void A_generic_type_matches_by_its_source_name()
        {
            Assert.True(AuditCommand.NamesType("new HoldLast<Pose>()", "HoldLast`1"));
        }

        [Fact]
        public void A_type_named_only_in_a_comment_is_not_registered()
        {
            string source = AuditCommand.StripLineComments("// TODO: register Kalman\n[\"cv\"] = c => new ConstantVelocity(c),");
            Assert.False(AuditCommand.NamesType(source, "Kalman"));
            Assert.True(AuditCommand.NamesType(source, "ConstantVelocity"));
        }
    }
}
