package frc.robot.subsystems.intake;

public record IntakeConfig (
    int intakeMotorID,
    int agitatorMotorID,
    int leftSolenoidForwardID,
    int leftSolenoidReverseID,
    int rightSolenoidForwardID,
    int rightSolenoidReverseID
) {}
