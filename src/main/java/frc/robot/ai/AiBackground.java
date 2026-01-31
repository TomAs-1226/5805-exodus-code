package frc.robot.ai;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Ultra-light "assist" that NEVER zeroes modules, NEVER holds a heading,
 * and NEVER injects a turn target. It only applies small, on-the-fly QoL
 * scalars to the already-shaped joystick commands:
 *
 *  - Pitch-based translation trim ("anti-tip"): scales X/Y when the robot is pitched.
 *  - Extra rotation deadband to fight tiny stick drift.
 *  - Slight rotation damping while translating to reduce skating at high speeds.
 *
 * All knobs are exposed on SmartDashboard under "AIbg/*".
 */
public final class AiBackground {

  /** Container for the assisted outputs. */
  public static final class Output {
    public final double x;
    public final double y;
    public final double rot;
    public Output(double x, double y, double rot) {
      this.x = x; this.y = y; this.rot = rot;
    }
  }

  private final DoubleSupplier pitchDeg;

  public AiBackground(DoubleSupplier yawDeg,
                      DoubleSupplier yawRateDegPerSec,
                      DoubleSupplier pitchDeg) {
    this.pitchDeg = pitchDeg;
  }

  /** Seed reasonable defaults to Shuffleboard if not already present. */
  public void initTunableDefaults() {
    putDefault("AIbg/TransScale",            1.00); // global scale on X/Y after your own slew
    putDefault("AIbg/PitchReduceStartDeg",   8.0);  // begin trimming translation here
    putDefault("AIbg/PitchReduceFullDeg",   14.0);  // full trim by this pitch
    putDefault("AIbg/PitchMinScale",         0.30); // min translation scale at/above full pitch
    putDefault("AIbg/RotWhileStrafeScale",   0.85); // scale rotation while translating (0.7..1.0)
    putDefault("AIbg/RotExtraDeadband",      0.06); // extra rot deadband AFTER your own shaping
  }

  /** Reset any internal temporal state (currently stateless but kept for future use). */
  public void onTeleopInit() {
    // Intentionally stateless to avoid any "return to a locked position" behavior.
  }

  /**
   * Apply gentle, one-shot QoL scalars to already-shaped x/y/rot.
   * This function is PURE (no persistence), so it cannot "pull" the robot
   * back to any prior heading or angle.
   */
  public Output apply(double xCmd, double yCmd, double rotCmd) {
    // --- Fetch knobs ---
    final double transScale = get("AIbg/TransScale", 1.00);
    final double startPitch = get("AIbg/PitchReduceStartDeg", 8.0);
    final double fullPitch  = get("AIbg/PitchReduceFullDeg", 14.0);
    final double minScale   = clip01(get("AIbg/PitchMinScale", 0.30));
    final double rotStrafeK = clip01(get("AIbg/RotWhileStrafeScale", 0.85));
    final double rotExtraDb = Math.abs(get("AIbg/RotExtraDeadband", 0.06));

    // --- Pitch-based translation trim (anti-tip style) ---
    final double pitch = Math.abs(pitchDeg.getAsDouble());
    final double pitchScale = (pitch <= startPitch)
        ? 1.0
        : (pitch >= fullPitch)
          ? minScale
          : lerp(1.0, minScale, (pitch - startPitch) / Math.max(1e-6, (fullPitch - startPitch)));

    double x = xCmd * transScale * pitchScale;
    double y = yCmd * transScale * pitchScale;

    // --- Rotation damping while translating (reduces skating at speed) ---
    final double transMag = Math.hypot(x, y); // 0..~1
    final double rotWhileMoveScale = lerp(1.0, rotStrafeK, clamp(transMag, 0.0, 1.0));
    double rot = rotCmd * rotWhileMoveScale;

    // --- Extra rotation deadband to crush tiny drift ---
    if (Math.abs(rot) < rotExtraDb) rot = 0.0;

    return new Output(x, y, rot);
  }

  // --------- small utils ----------
  private static void putDefault(String key, double val) {
    if (!SmartDashboard.containsKey(key)) SmartDashboard.putNumber(key, val);
  }
  private static double get(String key, double def) {
    return SmartDashboard.getNumber(key, def);
  }
  private static double lerp(double a, double b, double t01) {
    return a + clamp(t01, 0.0, 1.0) * (b - a);
  }
  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }
  private static double clip01(double v) {
    return clamp(v, 0.0, 1.0);
  }
}
