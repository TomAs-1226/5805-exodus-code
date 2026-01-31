package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Global, software-only azimuth bias.
 *
 * This rotates the commanded (forward, left) chassis vector by a constant angle
 * so that driver controls feel correct even if the physical azimuth zero appears
 * uniformly rotated. This does NOT replace proper CANcoder offset calibration.
 */
public final class TurnBias {
  /** Set this to 0.0 if you don't want any software rotation. Positive is CCW. */
  public static final double SOFT_AZIMUTH_BIAS_DEG = 90.0; // example: +90° (¼ turn)

  private TurnBias() {}

  /** Rotation2d for the bias (CCW positive). */
  public static Rotation2d rotationCCW() {
    return Rotation2d.fromDegrees(SOFT_AZIMUTH_BIAS_DEG);
  }

  /** Inverse rotation – use this to rotate your driver commands. */
  public static Rotation2d inverseRotationCCW() {
    return Rotation2d.fromDegrees(-SOFT_AZIMUTH_BIAS_DEG);
  }

  /** Rotate a 2D vector (x, y) by the provided Rotation2d. Returns {xr, yr}. */
  public static double[] rotate(double x, double y, Rotation2d rot) {
    final double c = rot.getCos();
    final double s = rot.getSin();
    final double xr =  x * c - y * s;
    final double yr =  x * s + y * c;
    return new double[] { xr, yr };
  }
}
