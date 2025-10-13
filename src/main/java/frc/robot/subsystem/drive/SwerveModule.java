package frc.robot.subsystem.drive;

import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.Constants;
import frc.robot.Constants.Device;
import frc.robot.device.KrakenX60;
import frc.robot.util.RobotMath;

/**
 * swerve module class blah blah u should never rlly need this
 * this code isnt pretty.. didnt care enough 2 make it better
 * undoubtably some wpilib class already does this all
 * @author florpy
 */
public class SwerveModule {
    
    /** the drive motor of this module */
    private final KrakenX60 driveMotor;
    /** the steer motor of this module */
    private final KrakenX60 steerMotor;
    /** the encoder (measures rotations & stuffs) of this module */
    private final CANcoder encoder;
    /** current target drive speed */
    private double targetDriveSpeed;
    /** current target angle */
    private double targetAngle;

    /**
     * makes a new swerve module with the specified motor/encoder stuffs <br>
     * (they hold ids u can find them in {@link Constants})
     * @param drive the drive motor
     * @param steer the steer motor
     * @param encoder the encoder motor
     * @see Constants.Motor
     * @see Constants.Encoder
     */
    public SwerveModule(Device drive, Device steer, Device encoder) {
        this.driveMotor = new KrakenX60(drive.ID, "Default Name");
        this.steerMotor = new KrakenX60(steer.ID, "Default Name");
        this.encoder = new CANcoder(encoder.ID, "Default Name");
        // Bump limits so steer can swing modules under ±45° quickly (prevents drive from being zeroed).
        setMaxMotorStates(0.60, 0.85); // drive 60%, steer 85% (safe defaults; still tamer than 100%)
        setTarget(0.0, 0.0);
    }

    /**
     * sets max motor states for this module
     * @param maxDrive max drive state [0.0, 1.0]
     * @param maxSteer max steer state [0.0, 1.0]
     */
    public void setMaxMotorStates(double maxDrive, double maxSteer) {
        setMaxDriveState(maxDrive);
        setMaxSteerState(maxSteer);
    }

    /**
     * sets max drive motor state
     * @param maxState max drive motor state
     */
    public void setMaxDriveState(double maxState) {
        this.driveMotor.setMaxState(maxState);
    }

    /**
     * sets max steer motor state
     * @param maxState max steer motor state
     */
    public void setMaxSteerState(double maxState) {
        this.steerMotor.setMaxState(maxState);
    }

    /**
     * gets the drive motor's max state
     * @return max drive motor state [0.0, 1.0]
     */
    public double getMaxDriveState() {
        return driveMotor.getMaxState();
    }

    /**
     * gets the steer motor's max state
     * @return max steer motor state [0.0, 1.0]
     */
    public double getMaxSteerState() {
        return steerMotor.getMaxState();
    }

    /**
     * sets motor speeds for both motors
     * @param driveSpeed drive motor speed
     * @param steerSpeed steer motor speed
     */
    public void setMotorSpeeds(double driveSpeed, double steerSpeed) {
        setDriveSpeed(driveSpeed);
        setSteerSpeed(steerSpeed);
    }

    /**
     * sets the drive motor speed
     * @param speed drive motor speed
     */
    public void setDriveSpeed(double speed) {
        this.driveMotor.set(speed);
    }

    /**
     * sets the steer motor speed
     * @param speed steer motor speed
     */
    public void setSteerSpeed(double speed) {
        this.steerMotor.set(speed);
    }

    /**
     * clears all sticky faults
     */
    public void clearFaults() {
        this.driveMotor.clearStickyFaults();
        this.steerMotor.clearStickyFaults();
        this.encoder.clearStickyFaults();
    }

    /**
     * sets target drive speed and angle
     * @param driveTarget drive motor target speed
     * @param angleTarget angle target
     */
    public void setTarget(double driveTarget, double angleTarget) {
        setTargetDriveSpeed(driveTarget);
        setTargetAngle(angleTarget);
    }

    /**
     * sets target drive speed
     * @param driveTarget drive target [-maxDriveState, maxDriveState]
     */
    public void setTargetDriveSpeed(double driveTarget) {
        double maxDrive = getMaxDriveState();
        this.targetDriveSpeed = RobotMath.clamp(driveTarget, -maxDrive, maxDrive);
    }

    /**
     * sets target angle
     * @param angle angle target (in degrees)
     */
    public void setTargetAngle(double angle) {
        this.targetAngle = RobotMath.clamp(angle, -360, 360);
    }

    /**
     * returns the angle reported by the encoder, from [0.0, 360.0)
     * @return angle of the module, from [0.0, 360)
     */
    public double getAngle() {
        double angle = getRawAngle();
        if (angle < 0.0) {
            angle = 360.0 - Math.abs(angle);
        }
        return angle;
    }

    /**
     * returns the raw angle measured by the cancoder
     * @return raw angle, in degrees, from [0.0, 180.0)
     */
    public double getRawAngle() {
        return this.encoder.getAbsolutePosition().getValueAsDouble() * 360.0;
    }

    /**
     * updates this motor autonomously to match the targetted <br> 
     * drive speed and motor angle
     */
    public void update() {
        double despeed = this.targetDriveSpeed;
        double deangle = this.targetAngle;
        if (RobotMath.getAngleAbsDiff(this.getAngle(), deangle) > 90) {
            deangle += 180;
            despeed *= -1;
        }
        double err = RobotMath.getSignedAngleDiff(this.getAngle(), deangle);
        double calcThrottle = -err / 360.0;
        if (Math.abs(err) > 45.0) {
            despeed = 0.0;
        }
        setMotorSpeeds(despeed, calcThrottle);
    }

}
