package frc.robot.subsystem.drive;

import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.Constants;
import frc.robot.Constants.Device;
import frc.robot.device.KrakenX60;
import frc.robot.util.RobotMath;

/**
 * Swerve module: drive + steer + absolute encoder.
 * Subtle QoL: "hold last angle" when requested drive speed is near zero
 * to prevent steer from snapping to a default direction at idle.
 */
public class SwerveModule {

    // the drive motor of this module
    private final KrakenX60 driveMotor;
    // the steer motor of this module
    private final KrakenX60 steerMotor;
    // the absolute encoder of this module
    private final CANcoder encoder;

    // command targets
    private double targetDriveSpeed;
    private double targetAngleDeg;

    // remember the last commanded angle for "hold on zero speed"
    private double lastAngleDeg = 0.0;

    // how small the requested drive must be before we hold angle
    // 0.04 = 4% of full command (tunable, but intentionally subtle)
    private static final double ANGLE_HOLD_MIN_DRIVE = 0.04;

    /**
     * Make a new swerve module with device IDs provided in Constants.
     */
    public SwerveModule(Device drive, Device steer, Device encoder) {
        this.driveMotor = new KrakenX60(drive.ID, "Default Name");
        this.steerMotor = new KrakenX60(steer.ID, "Default Name");
        this.encoder    = new CANcoder(encoder.ID, "Default Name");
        // Tame defaults (safer on carpet)
        setMaxMotorStates(0.60, 0.85); // drive 60%, steer 85%
        setTarget(0.0, getAngle());    // start by holding current physical angle
        this.lastAngleDeg = getAngle();
    }

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
    public double getMaxDriveState() {
        return driveMotor.getMaxState();
    }

    /** gets the steer motor's max state */
    public double getMaxSteerState() {
        return steerMotor.getMaxState();
    }

    /** sets motor speeds directly */
    public void setMotorSpeeds(double driveSpeed, double steerSpeed) {
        setDriveSpeed(driveSpeed);
        setSteerSpeed(steerSpeed);
    }

    public void setDriveSpeed(double speed) { this.driveMotor.set(speed); }
    public void setSteerSpeed(double speed) { this.steerMotor.set(speed); }

    /** clears all sticky faults */
    public void clearFaults() {
        this.driveMotor.clearStickyFaults();
        this.steerMotor.clearStickyFaults();
        this.encoder.clearStickyFaults();
    }

    /**
     * Set target drive speed and angle (deg).
     * Subtle assist: if drive command is tiny, hold the last angle instead of
     * chasing a new angle. This prevents "snap to straight" at idle.
     */
    public void setTarget(double driveTarget, double angleTargetDeg) {
        // drive clamp
        final double maxDrive = getMaxDriveState();
        final double clampedDrive = RobotMath.clamp(driveTarget, -maxDrive, maxDrive);

        // decide which angle to use
        final boolean holdAngle = Math.abs(clampedDrive) < ANGLE_HOLD_MIN_DRIVE;
        final double chosenAngleDeg = holdAngle ? lastAngleDeg
                                                : RobotMath.clamp(angleTargetDeg, -360.0, 360.0);

        // store targets
        this.targetDriveSpeed = clampedDrive;
        this.targetAngleDeg   = chosenAngleDeg;

        // if we actually updated angle (moving meaningfully), refresh lastAngleDeg
        if (!holdAngle) {
            this.lastAngleDeg = chosenAngleDeg;
        }
    }

    /** current module angle [0, 360) from cancoder absolute */
    public double getAngle() {
        double angle = getRawAngle();
        if (angle < 0.0) {
            angle = 360.0 - Math.abs(angle);
        }
        return angle;
    }

    /** raw cancoder absolute angle in degrees (0..360) */
    public double getRawAngle() {
        return this.encoder.getAbsolutePosition().getValueAsDouble() * 360.0;
    }

    /**
     * Core periodic control: steer toward target angle and drive requested speed.
     * - If the desired angle is more than 90 deg away, flip by 180 and invert drive.
     * - When far off (>45 deg), pause drive to avoid skating.
     */
    public void update() {
        double driveCmd = this.targetDriveSpeed;
        double goalDeg  = this.targetAngleDeg;

        // 180-optimize the path to reduce rotation
        if (RobotMath.getAngleAbsDiff(this.getAngle(), goalDeg) > 90.0) {
            goalDeg += 180.0;
            driveCmd *= -1.0;
        }

        // steer error and simple proportional steer throttle
        double errDeg = RobotMath.getSignedAngleDiff(this.getAngle(), goalDeg);
        double steerThrottle = -errDeg / 360.0;

        // When we're far off (>45 deg), don't drive the wheel to avoid skating
        if (Math.abs(errDeg) > 45.0) {
            driveCmd = 0.0;
        }

        setMotorSpeeds(driveCmd, steerThrottle);
    }
}
