package frc.robot.subsystem.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;

import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystem.AbstractSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Phoenix 6-backed swerve drivetrain wrapper that preserves the legacy AbstractSubsystem API.
 * Convention:
 *   forward  ( +X ) is field/robot forward
 *   strafe   ( +right stick ) -> +strafe -> robot +Y is left, so we internally negate to -Y
 *   omegaCCW ( + ) counter-clockwise
 */
public final class SwerveDrivetrain extends AbstractSubsystem {
    private static final double DEFAULT_MAX_ANGULAR_RATE_RAD_PER_SEC =
        RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final CommandSwerveDrivetrain drivetrain;
    private final Pigeon2 gyro;
    private final Telemetry telemetry;

    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    private final double maxSpeedMetersPerSecond =
        TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private double driveOutputScale = 0.60;
    private double steerOutputScale = 0.85;

    public SwerveDrivetrain() {
        this.drivetrain = TunerConstants.createDrivetrain();
        this.gyro = drivetrain.getPigeon2();
        this.telemetry = new Telemetry(maxSpeedMetersPerSecond);
        drivetrain.registerTelemetry(telemetry::telemeterize);
        drivetrain.seedFieldCentric();
    }

    @Override
    public void start() {
        applyIdle();
    }

    @Override
    public void update() {
        // Telemetry is handled internally by Phoenix when registered above.
    }

    @Override
    public void stop() {
        applyIdle();
    }

    /** Zero yaw to 0 degrees (also reseeds FieldCentric). */
    public void zeroGyro() {
        setYawDegrees(0.0);
    }

    /** Set yaw to an explicit angle in degrees and reseed field-centric. */
    public void setYawDegrees(double degrees) {
        gyro.setYaw(degrees);
        drivetrain.seedFieldCentric();
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
     * Core drive:
     *  strafe: + to the RIGHT on the stick (we map to -Y internally)
     *  forward: + forward
     *  omegaCCW: + CCW
     *  foc: true = field-oriented, false = robot-centric
     */
    public void drive(double strafe, double forward, double omegaCCW, boolean foc) {
        double vxMeters     = forward * maxSpeedMetersPerSecond * driveOutputScale;   // +X forward
        double vyMeters     = -strafe * maxSpeedMetersPerSecond * driveOutputScale;   // stick right => -Y
        double omegaRadians =  omegaCCW * DEFAULT_MAX_ANGULAR_RATE_RAD_PER_SEC * steerOutputScale; // +CCW

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

    private void applyIdle() {
        drivetrain.setControl(idleRequest);
    }
}
