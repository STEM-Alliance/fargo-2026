package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

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

    public final void setRunning(boolean running) {
        if (running) {
            m_intakeIO.setIntakeMotorVoltage(Volts.of(12.0));
        } else {
            m_intakeIO.setIntakeMotorVoltage(Volts.zero());
        }
    }

    public final void setExtended(boolean extended) {
        m_intakeIO.setIntakeExtended(extended);
    }

    public final void toggleIntakeExtended() {
        m_intakeIO.setIntakeExtended(!m_intakeInputs.isIntakeSolenoidForward);
    }
}
