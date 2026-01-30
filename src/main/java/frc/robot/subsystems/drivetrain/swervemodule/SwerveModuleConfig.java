package frc.robot.subsystems.drivetrain.swervemodule;

import edu.wpi.first.math.geometry.Translation2d;

public record SwerveModuleConfig(
    int driveMotorID,
    int azimuthMotorID,
    int azimuthEncoderID,
    Translation2d moduleTranslation
) {}
