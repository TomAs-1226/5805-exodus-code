package frc.robot.subsystem.drive;

import java.util.HashMap;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Device;
import frc.robot.subsystem.AbstractSubsystem;
import frc.robot.util.RobotMath;
import frc.robot.util.Vec2;

/**
 * Swerve drivetrain (WPILib field-relative -> your original str/fwd vector math).
 */
public final class SwerveDrivetrain extends AbstractSubsystem {

    private static final String CANBUS = "*"; // your CANivore (name or "*" works)

    // SmartDashboard encoder offsets (deg) so 0° = robot-forward per module
    private static final String OFF_FL_KEY = "Swerve/OffsetDeg/FL";
    private static final String OFF_FR_KEY = "Swerve/OffsetDeg/FR";
    private static final String OFF_BL_KEY = "Swerve/OffsetDeg/BL";
    private static final String OFF_BR_KEY = "Swerve/OffsetDeg/BR";

    private final HashMap<String, SwerveModule> MODULES;
    private final Pigeon2 GYRO;

    public SwerveDrivetrain() {
        SmartDashboard.setDefaultNumber(OFF_FL_KEY, 0.0);
        SmartDashboard.setDefaultNumber(OFF_FR_KEY, 0.0);
        SmartDashboard.setDefaultNumber(OFF_BL_KEY, 0.0);
        SmartDashboard.setDefaultNumber(OFF_BR_KEY, 0.0);

        this.MODULES = new HashMap<>();
        MODULES.put("FL", new SwerveModule(Device.FL_DRIVE, Device.FL_SWERVE, Device.FL_ENCODER, CANBUS, SmartDashboard.getNumber(OFF_FL_KEY, 0.0)));
        MODULES.put("FR", new SwerveModule(Device.FR_DRIVE, Device.FR_SWERVE, Device.FR_ENCODER, CANBUS, SmartDashboard.getNumber(OFF_FR_KEY, 0.0)));
        MODULES.put("BL", new SwerveModule(Device.BL_DRIVE, Device.BL_SWERVE, Device.BL_ENCODER, CANBUS, SmartDashboard.getNumber(OFF_BL_KEY, 0.0)));
        MODULES.put("BR", new SwerveModule(Device.BR_DRIVE, Device.BR_SWERVE, Device.BR_ENCODER, CANBUS, SmartDashboard.getNumber(OFF_BR_KEY, 0.0)));

        this.GYRO = new Pigeon2(Device.PIGEON_2.ID, CANBUS);
    }

    public void clearFaults() {
        for (SwerveModule module : MODULES.values()) {
            module.clearFaults();
        }
        GYRO.clearStickyFaults();
    }

    /** Zero yaw to 0 deg (CCW+). */
    public void zeroGyro() { GYRO.setYaw(0); }

    /** Set yaw to explicit angle (deg). */
    public void setYawDegrees(double deg) { GYRO.setYaw(deg); }

    /** Re-orient for facing driver station. */
    public void orientFacingDriverStation() { setYawDegrees(180.0); }

    public void setDriveMaxAll(double max) { for (SwerveModule m : MODULES.values()) m.setMaxDriveState(max); }
    public void setSteerMaxAll(double max) { for (SwerveModule m : MODULES.values()) m.setMaxSteerState(max); }

    /**
     * Drive with your original semantics:
     *   str = left/right, fwd = forward/back, omega = rotation.
     * We do WPILib field-relative transform, then map back to (str,fwd).
     */
    public void drive(double str, double fwd, double omega, boolean foc) {
        // Idle hold to prevent dither when stopped
        if (Math.abs(str) + Math.abs(fwd) + Math.abs(omega) < 1e-6) {
            MODULES.get("FR").setTarget(0.0, MODULES.get("FR").getAngle());
            MODULES.get("FL").setTarget(0.0, MODULES.get("FL").getAngle());
            MODULES.get("BR").setTarget(0.0, MODULES.get("BR").getAngle());
            MODULES.get("BL").setTarget(0.0, MODULES.get("BL").getAngle());
            return;
        }
        // Restore your driver feel:
        str   *= -1; // you had this
        omega *= -1; // restore your previous rotation direction
        // Field-relative or robot-relative chassis speeds (WPILib: X=fwd, Y=left, CCW+). 
        // We’ll convert back to (str,fwd) for your Vec2 math. 
        ChassisSpeeds speeds;
        if (foc) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(fwd, str, omega, GYRO.getRotation2d());
        } else {
            speeds = new ChassisSpeeds(fwd, str, omega);
        }

        // Second-order discretization to reduce skew at high omega.
        speeds = ChassisSpeeds.discretize(
            speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond, Robot.kDefaultPeriod
        );

        // Map WPILib (vx=fwd, vy=left) back to your original naming (str first, fwd second):
        final double strR = speeds.vyMetersPerSecond; // your Vec2 X = strafe
        final double fwdR = speeds.vxMetersPerSecond; // your Vec2 Y = forward
        final double wz   = speeds.omegaRadiansPerSecond;

        // Your original A/B/C/D layout expects (first = str, second = fwd)
        double r  = Math.hypot(Constants.MODULE_LENGTH, Constants.MODULE_WIDTH);
        double lr = Constants.MODULE_LENGTH / r;
        double lw = Constants.MODULE_WIDTH  / r;

        double a = strR - wz * lr;
        double b = strR + wz * lr;
        double c = fwdR - wz * lw;
        double d = fwdR + wz * lw;

        Vec2 fr = new Vec2(b, c);
        Vec2 fl = new Vec2(b, d);
        Vec2 bl = new Vec2(a, d);
        Vec2 br = new Vec2(a, c);

        double max = RobotMath.maxOf(fr.mag(), fl.mag(), bl.mag(), br.mag());
        double scale = (max > 1.0) ? (1.0 / max) : 1.0;

        MODULES.get("FR").setTarget(fr.mag() * scale, fr.toAngle());
        MODULES.get("FL").setTarget(fl.mag() * scale, fl.toAngle());
        MODULES.get("BR").setTarget(br.mag() * scale, br.toAngle());
        MODULES.get("BL").setTarget(bl.mag() * scale, bl.toAngle());
    }

    @Override
    public void start() {
        clearFaults();
        // Reload offsets on enable so you can tweak in SD then re-enable
        MODULES.get("FL").setEncoderOffsetDeg(SmartDashboard.getNumber(OFF_FL_KEY, 0.0));
        MODULES.get("FR").setEncoderOffsetDeg(SmartDashboard.getNumber(OFF_FR_KEY, 0.0));
        MODULES.get("BL").setEncoderOffsetDeg(SmartDashboard.getNumber(OFF_BL_KEY, 0.0));
        MODULES.get("BR").setEncoderOffsetDeg(SmartDashboard.getNumber(OFF_BR_KEY, 0.0));
    }

    @Override
    public void update() {
        for (SwerveModule module : MODULES.values()) {
            module.update();
        }
        MODULES.get("FL").publishTelemetry("FL");
        MODULES.get("FR").publishTelemetry("FR");
        MODULES.get("BL").publishTelemetry("BL");
        MODULES.get("BR").publishTelemetry("BR");
    }

    @Override
    public void stop() { drive(0, 0, 0, false); }
}
