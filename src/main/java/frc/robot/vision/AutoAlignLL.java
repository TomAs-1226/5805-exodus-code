package frc.robot.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * Minimal Limelight auto-align helper: only strafe (x) + rotate (omega).
 * - Uses targetpose_robotspace when available for proper lateral meters + yaw.
 * - Falls back to tx-only if 3D solve isn't available.
 * - No forward/back output.
 * - LEDs forced on while enabled; pipeline forced to the configured index.
 */
public final class AutoAlignLL {

  public static final class Output {
    public final double strafe;   // left(+)/right(-), robot-relative
    public final double omega;    // CCW(+), CW(-)
    public final boolean hasTarget;
    public Output(double s, double o, boolean h) { this.strafe = s; this.omega = o; this.hasTarget = h; }
  }

  private final String name;
  private final int pipelineIndex;

  // Tunables (kept modest; yaw a bit firmer so you can see turning)
  private static final double Y_DB_METERS   = 0.03;  // ignore tiny lateral error
  private static final double Y_KP          = 1.25;  // strafe m->cmd
  private static final double Y_CMD_MAX     = 0.45;  // clamp

  private static final double YAW_DB_DEG    = 1.0;   // ignore tiny yaw
  private static final double YAW_KP_DEG    = 0.060; // deg->cmd (slightly firmer than before)
  private static final double YAW_CMD_MAX   = 0.70;

  // Fallback (tx-only) — still subtle
  private static final double TX_STR_KP     = 0.020; // deg->strafe
  private static final double TX_YAW_KP     = 0.040; // deg->omega (slightly firmer)

  private boolean enabled = false;

  public AutoAlignLL(String limelightName, int pipelineIndex) {
    this.name = limelightName;
    this.pipelineIndex = pipelineIndex;
  }

  public void enable() {
    if (!enabled) {
      enabled = true;
      LimelightHelpers.setPipelineIndex(name, pipelineIndex);
      LimelightHelpers.setLEDMode_ForceOn(name);
    }
  }

  public void disable() {
    if (enabled) {
      enabled = false;
      // Return LEDs/pipeline control to normal pipeline behavior
      LimelightHelpers.setLEDMode_PipelineControl(name);
    }
  }

  public boolean isEnabled() { return enabled; }

  /** Compute one step of strafe/yaw. No forward component is produced. */
  public Output update() {
    if (!enabled) return new Output(0.0, 0.0, false);

    boolean tv = LimelightHelpers.getTV(name);
    if (!tv) return new Output(0.0, 0.0, false);

    // Prefer 3D robot-space pose when available (AprilTag 3D pipeline)
    Pose3d tpr = LimelightHelpers.getTargetPose3d_RobotSpace(name);
    double yMeters = tpr.getTranslation().getY();     // +left, -right (robot space)
    double yawRad  = tpr.getRotation().getZ();        // radians (robot yaw error to tag)
    boolean have3D = !(Double.isNaN(yMeters) || Double.isNaN(yawRad));

    double strafeCmd = 0.0;
    double omegaCmd  = 0.0;

    if (have3D) {
      double yawDeg = Math.toDegrees(yawRad);

      if (Math.abs(yMeters) > Y_DB_METERS) {
        strafeCmd = MathUtil.clamp(Y_KP * yMeters, -Y_CMD_MAX, Y_CMD_MAX);
      }
      if (Math.abs(yawDeg) > YAW_DB_DEG) {
        omegaCmd = MathUtil.clamp(YAW_KP_DEG * yawDeg, -YAW_CMD_MAX, YAW_CMD_MAX);
      }
      return new Output(strafeCmd, omegaCmd, true);
    }

    // Fallback: tx-only (deg). Still subtle; might be less precise laterally.
    double tx = LimelightHelpers.getTX(name);
    if (Math.abs(tx) < 1e-3) {
      return new Output(0.0, 0.0, true);
    }
    strafeCmd = MathUtil.clamp(TX_STR_KP * tx, -Y_CMD_MAX, Y_CMD_MAX);
    omegaCmd  = MathUtil.clamp(TX_YAW_KP * tx, -YAW_CMD_MAX, YAW_CMD_MAX);
    return new Output(strafeCmd, omegaCmd, true);
  }
}
