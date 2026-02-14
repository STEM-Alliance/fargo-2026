package frc.robot.subsystems.shooter.turret;

public record TurretConfig(
    int turretMotorID,
    int turretEncoderID,
    int hoodMotorID,
    int hoodHomingSwitchID
) {}
