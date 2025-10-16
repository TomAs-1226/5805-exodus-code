package frc.robot.subsystem.drive;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.Device;
import frc.robot.device.KrakenX60;
import frc.robot.util.RobotMath;

/**
 * Swerve module with absolute encoder offset + gentler steer loop.
 */
public class SwerveModule {

    /** the drive motor of this module */
    private final KrakenX60 driveMotor;
    /** the steer motor of this module */
    private final KrakenX60 steerMotor;
    /** the absolute encoder for module azimuth */
    private final CANcoder encoder;

    /** absolute-angle offset (deg) so that 0° = robot-forward for THIS module */
    private double encoderOffsetDeg;

    /** current target drive speed */
    private double targetDriveSpeed;
    /** current target angle (deg, 0..360) in robot frame */
    private double targetAngle;

    /**
     * @param drive    drive motor device id wrapper
     * @param steer    steer motor device id wrapper
     * @param encoder  cancoder device id wrapper
     * @param canbus   CAN bus string (e.g. "*" or your CANivore name)
     * @param encoderOffsetDeg absolute offset (deg) so module "straight" is 0°
     */
    public SwerveModule(Device drive, Device steer, Device encoder, String canbus, double encoderOffsetDeg) {
        this.driveMotor = new KrakenX60(drive.ID, canbus);
        this.steerMotor = new KrakenX60(steer.ID, canbus);
        this.encoder    = new CANcoder(encoder.ID, canbus);
        this.encoderOffsetDeg = encoderOffsetDeg;

        // Tame defaults: steer has authority to move, drive modest.
        setMaxMotorStates(0.60, 0.85); // drive 60%, steer 85%
        setTarget(0.0, 0.0);
    }

    /** Allow reloading offset at enable. */
    public void setEncoderOffsetDeg(double deg) { this.encoderOffsetDeg = deg; }

    /** sets max motor states for this module */
    public void setMaxMotorStates(double maxDrive, double maxSteer) {
        setMaxDriveState(maxDrive);
        setMaxSteerState(maxSteer);
    }

    /** sets max drive motor state */
    public void setMaxDriveState(double maxState) {
        this.driveMotor.setMaxState(maxState);
    }

    /** sets max steer motor state */
    public void setMaxSteerState(double maxState) {
        this.steerMotor.setMaxState(maxState);
    }

    /** gets the drive motor's max state */
    public double getMaxDriveState() { return driveMotor.getMaxState(); }
    /** gets the steer motor's max state */
    public double getMaxSteerState() { return steerMotor.getMaxState(); }

    /** sets motor speeds */
    public void setMotorSpeeds(double driveSpeed, double steerSpeed) {
        setDriveSpeed(driveSpeed);
        setSteerSpeed(steerSpeed);
    }

    /** sets the drive motor speed */
    public void setDriveSpeed(double speed) { this.driveMotor.set(speed); }

    /** sets the steer motor speed */
    public void setSteerSpeed(double speed) { this.steerMotor.set(speed); }

    /** clears all sticky faults */
    public void clearFaults() {
        this.driveMotor.clearStickyFaults();
        this.steerMotor.clearStickyFaults();
        this.encoder.clearStickyFaults();
    }

    /** sets target drive speed and angle */
    public void setTarget(double driveTarget, double angleTarget) {
        setTargetDriveSpeed(driveTarget);
        setTargetAngle(angleTarget);
    }

    /** sets target drive speed */
    public void setTargetDriveSpeed(double driveTarget) {
        double maxDrive = getMaxDriveState();
        this.targetDriveSpeed = RobotMath.clamp(driveTarget, -maxDrive, maxDrive);
    }

    /** sets target angle (deg) */
    public void setTargetAngle(double angle) {
        // normalize into [0, 360)
        double a = angle % 360.0;
        if (a < 0) a += 360.0;
        this.targetAngle = a;
    }

    /** Absolute raw angle from CANcoder (deg 0..360). */
    public double getRawAngle() {
        return this.encoder.getAbsolutePosition().getValueAsDouble() * 360.0;
    }

    /** Calibrated angle = raw - offset  (deg 0..360). */
    public double getAngle() {
        double angle = getRawAngle() - encoderOffsetDeg;
        angle %= 360.0;
        if (angle < 0) angle += 360.0;
        return angle;
    }

    /** Update towards target. */
    public void update() {
        double despeed = this.targetDriveSpeed;
        double deangle = this.targetAngle;

        // If >90° away, flip 180° and drive backwards (shortest rotation).
        if (RobotMath.getAngleAbsDiff(this.getAngle(), deangle) > 90) {
            deangle += 180.0;
            despeed *= -1.0;
        }

        double err = RobotMath.getSignedAngleDiff(this.getAngle(), deangle);

        // Gentler steer: smaller proportional (was -err/360). Clamp to max steer authority.
        double calcThrottle = RobotMath.clamp(-err / 180.0, -getMaxSteerState(), getMaxSteerState());

        // If we’re really far off, stop drive to avoid scrub.
        if (Math.abs(err) > 45.0) {
            despeed = 0.0;
        }

        setMotorSpeeds(despeed, calcThrottle);
    }

    /** Helpful telemetry for offset calibration. */
    public void publishTelemetry(String tag) {
        SmartDashboard.putNumber("Swerve/" + tag + "/AbsRawDeg", getRawAngle());
        SmartDashboard.putNumber("Swerve/" + tag + "/AngleDeg",  getAngle());
        SmartDashboard.putNumber("Swerve/" + tag + "/OffsetDeg", encoderOffsetDeg);
    }
}
