package frc.robot.subsystems;

import java.util.function.Consumer;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Lightweight wrapper around the generated Phoenix 6 drivetrain.
 * This keeps the generated API surface available without pulling in
 * the entire command-based infrastructure from the example project.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain {
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1> odometryStandardDeviation,
        edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
    }

    public void registerTelemetry(Consumer<SwerveDriveState> telemetryConsumer) {
        super.registerTelemetry(telemetryConsumer);
    }
}
