package frc.robot.subsystem.drive;

import java.util.HashMap;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Device;
import frc.robot.subsystem.AbstractSubsystem;
import frc.robot.util.RobotMath;
import frc.robot.util.Vec2;

/**
 * Simple swerve drivetrain.
 * QoL only: no auto-zeroing during teleop. We rely on SwerveModule's
 * "hold last angle" behavior at very low speeds to avoid snap-to-straight.
 */
public final class SwerveDrivetrain extends AbstractSubsystem {

    private final HashMap<String, SwerveModule> MODULES;
    private final Pigeon2 GYRO;

    public SwerveDrivetrain() {
        this.MODULES = new HashMap<>();
        MODULES.put("FL", new SwerveModule(Device.FL_DRIVE, Device.FL_SWERVE, Device.FL_ENCODER));
        MODULES.put("FR", new SwerveModule(Device.FR_DRIVE, Device.FR_SWERVE, Device.FR_ENCODER));
        MODULES.put("BL", new SwerveModule(Device.BL_DRIVE, Device.BL_SWERVE, Device.BL_ENCODER));
        MODULES.put("BR", new SwerveModule(Device.BR_DRIVE, Device.BR_SWERVE, Device.BR_ENCODER));
        this.GYRO = new Pigeon2(Device.PIGEON_2.ID, "Default Name");
    }

    public void clearFaults() {
        for (SwerveModule module : MODULES.values()) {
            module.clearFaults();
        }
        GYRO.clearStickyFaults();
    }

    /** Zero yaw to 0 deg */
    public void zeroGyro() {
        GYRO.setYaw(0);
    }

    /** Set yaw to an explicit angle (deg) */
    public void setYawDegrees(double deg) {
        GYRO.setYaw(deg);
    }

    /** Field re-orient for starts facing the driver station (180 deg). */
    public void orientFacingDriverStation() {
        setYawDegrees(180.0);
    }

    /** Helper: set drive max for all modules (0..1) */
    public void setDriveMaxAll(double max) {
        for (SwerveModule m : MODULES.values()) m.setMaxDriveState(max);
    }

    /** Helper: set steer max for all modules (0..1) */
    public void setSteerMaxAll(double max) {
        for (SwerveModule m : MODULES.values()) m.setMaxSteerState(max);
    }

    /**
     * Core drive: strafe (x), forward (y), and omega (CCW+).
     * 'foc' enables field-oriented control using gyro yaw.
     */
    public void drive(double str, double fwd, double omega, boolean foc) {
        // respect team coordinate convention
        str *= -1;
        omega *= -1;

        if (foc) {
            double theta = -GYRO.getYaw().getValueAsDouble() * (Math.PI / 180.0);
            double tmp = fwd * Math.cos(theta) + str * Math.sin(theta);
            str = -fwd * Math.sin(theta) + str * Math.cos(theta);
            fwd = tmp;
        }

        ChassisSpeeds speeds = ChassisSpeeds.discretize(str, fwd, omega, Robot.kDefaultPeriod);
        str   = speeds.vxMetersPerSecond;
        fwd   = speeds.vyMetersPerSecond;
        omega = speeds.omegaRadiansPerSecond;

        double r  = Math.hypot(Constants.MODULE_LENGTH, Constants.MODULE_WIDTH);
        double lr = Constants.MODULE_LENGTH / r;
        double lw = Constants.MODULE_WIDTH / r;

        double a = str - omega * lr;
        double b = str + omega * lr;
        double c = fwd - omega * lw;
        double d = fwd + omega * lw;

        Vec2 fr = new Vec2(b, c);
        Vec2 fl = new Vec2(b, d);
        Vec2 bl = new Vec2(a, d);
        Vec2 br = new Vec2(a, c);

        double max = RobotMath.maxOf(fr.mag(), fl.mag(), bl.mag(), br.mag());
        if (max > 1.0) {
            double rat = 1.0 / max;
            MODULES.get("FR").setTarget(fr.mag() * rat, fr.toAngle());
            MODULES.get("FL").setTarget(fl.mag() * rat, fl.toAngle());
            MODULES.get("BR").setTarget(br.mag() * rat, br.toAngle());
            MODULES.get("BL").setTarget(bl.mag() * rat, bl.toAngle());
        } else {
            MODULES.get("FR").setTarget(fr.mag(), fr.toAngle());
            MODULES.get("FL").setTarget(fl.mag(), fl.toAngle());
            MODULES.get("BR").setTarget(br.mag(), br.toAngle());
            MODULES.get("BL").setTarget(bl.mag(), bl.toAngle());
        }
    }

    @Override
    public void start() {
        clearFaults();
        // No auto-steer "zeroing" here; Robot handles initial orientation only.
    }

    @Override
    public void update() {
        for (SwerveModule module : MODULES.values()) {
            module.update();
        }
    }

    @Override
    public void stop() {
        drive(0, 0, 0, false);
    }
}
