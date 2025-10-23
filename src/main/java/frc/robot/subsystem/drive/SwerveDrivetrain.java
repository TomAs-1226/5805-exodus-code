package frc.robot.subsystem.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;

import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystem.AbstractSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Phoenix 6-backed swerve drivetrain wrapper that preserves the legacy
 * {@link AbstractSubsystem} API used throughout the robot project.
 */
public final class SwerveDrivetrain extends AbstractSubsystem {
    private static final double DEFAULT_MAX_ANGULAR_RATE_RAD_PER_SEC =
        RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final CommandSwerveDrivetrain drivetrain;
    private final Pigeon2 gyro;
    private final Telemetry telemetry;

    private final SwerveRequest.FieldCentric fieldCentricRequest =
        new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric robotCentricRequest =
        new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    private final double maxSpeedMetersPerSecond =
        TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private double driveOutputScale = 1.0;
    private double steerOutputScale = 1.0;

    public SwerveDrivetrain() {
        this.drivetrain = TunerConstants.createDrivetrain();
        this.gyro = drivetrain.getPigeon2();
        this.telemetry = new Telemetry(maxSpeedMetersPerSecond);
        drivetrain.registerTelemetry(telemetry::telemeterize);
        drivetrain.seedFieldCentric();
    }

    @Override
    public void start() {
        drivetrain.setControl(idleRequest);
    }

    @Override
    public void update() {
        // Telemetry is handled internally by Phoenix when registered above.
    }

    @Override
    public void stop() {
        drivetrain.setControl(idleRequest);
    }

    /** Zero yaw to 0 degrees. */
    public void zeroGyro() {
        setYawDegrees(0.0);
    }

    /** Set yaw to an explicit angle in degrees. */
    public void setYawDegrees(double degrees) {
        gyro.setYaw(degrees);
        drivetrain.seedFieldCentric();
    }

    /** Field re-orient for starts facing the driver station (180 deg). */
    public void orientFacingDriverStation() {
        setYawDegrees(180.0);
    }

    /** Helper: set drive max for all modules (0..1). */
    public void setDriveMaxAll(double max) {
        driveOutputScale = MathUtil.clamp(max, 0.0, 1.0);
    }

    /** Helper: set steer max for all modules (0..1). */
    public void setSteerMaxAll(double max) {
        steerOutputScale = MathUtil.clamp(max, 0.0, 1.0);
    }

    /**
     * Core drive: strafe (x), forward (y), and omega (CCW+).
     * 'foc' enables field-oriented control using gyro yaw.
     */
    public void drive(double strafe, double forward, double omega, boolean foc) {
        double strCmd = -strafe;
        double fwdCmd = forward;
        double omegaCmd = -omega;

        if (foc) {
            double yawRad = Math.toRadians(gyro.getYaw().getValueAsDouble());
            double theta = -yawRad;
            double tmp = fwdCmd * Math.cos(theta) + strCmd * Math.sin(theta);
            strCmd = -fwdCmd * Math.sin(theta) + strCmd * Math.cos(theta);
            fwdCmd = tmp;
        }

        ChassisSpeeds speeds = ChassisSpeeds.discretize(
            strCmd,
            fwdCmd,
            omegaCmd,
            TimedRobot.kDefaultPeriod
        );

        double vxMeters = speeds.vxMetersPerSecond * maxSpeedMetersPerSecond * driveOutputScale;
        double vyMeters = speeds.vyMetersPerSecond * maxSpeedMetersPerSecond * driveOutputScale;
        double omegaRadians = speeds.omegaRadiansPerSecond *
            DEFAULT_MAX_ANGULAR_RATE_RAD_PER_SEC * steerOutputScale;

        if (foc) {
            drivetrain.setControl(fieldCentricRequest
                .withVelocityX(vxMeters)
                .withVelocityY(vyMeters)
                .withRotationalRate(omegaRadians));
        } else {
            drivetrain.setControl(robotCentricRequest
                .withVelocityX(vxMeters)
                .withVelocityY(vyMeters)
                .withRotationalRate(omegaRadians));
        }
    }
}
