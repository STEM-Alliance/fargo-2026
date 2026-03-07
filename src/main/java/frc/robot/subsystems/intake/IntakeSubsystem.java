package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.subsystems.intake.io.IntakeIO;
import frc.robot.subsystems.intake.io.IntakeInputsAutoLogged;

public class IntakeSubsystem implements Subsystem {
    private final IntakeIO m_intakeIO;
    private final IntakeInputsAutoLogged m_intakeInputs;

    public IntakeSubsystem(IntakeIO intakeIO) {
        m_intakeIO = intakeIO;
        m_intakeInputs = new IntakeInputsAutoLogged();
    }

    @Override
    public final void periodic() {
        m_intakeIO.updateInputs(m_intakeInputs);
        Logger.processInputs("IntakeSubsystem", m_intakeInputs);
    }

    public final void startIntake() {
        m_intakeIO.setIntakeMotorVoltage(Volts.of(-7.0));
    }

    public final void stopIntake() {
        m_intakeIO.setIntakeMotorVoltage(Volts.zero());
    }

    public final void setIntakeExtended(boolean extended) {
        m_intakeIO.setIntakeExtended(extended);
    }

    public final void toggleIntakeExtended() {
        m_intakeIO.setIntakeExtended(!m_intakeInputs.isIntakeExtended);
    }

    public final Command getAgitateFuelCommand() {
        return Commands.repeatingSequence(
            Commands.runOnce(this::toggleIntakeExtended),
            Commands.waitSeconds(0.25),
            Commands.runOnce(this::toggleIntakeExtended),
            Commands.waitSeconds(0.50)
        );
    }
}
