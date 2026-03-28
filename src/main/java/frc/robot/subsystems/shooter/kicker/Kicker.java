package frc.robot.subsystems.shooter.kicker;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.subsystems.shooter.kicker.io.KickerIO;
import frc.robot.subsystems.shooter.kicker.io.KickerInputsAutoLogged;

public final class Kicker implements Subsystem {
    private final KickerIO m_kickerIO;
    private final KickerInputsAutoLogged m_kickerInputs;

    public Kicker(KickerIO kickerIO) {
        m_kickerIO = kickerIO;
        m_kickerInputs = new KickerInputsAutoLogged();
    }

    public final void periodic() {
        m_kickerIO.updateInputs(m_kickerInputs);
        Logger.processInputs("ShooterSubsystem/Kicker", m_kickerInputs);
    }

    public final void setRunning(boolean running, boolean reverse) {
        m_kickerIO.setKickerMotorVoltage(
            running ? Volts.of(reverse ? -12.0 : 12.0) : Volts.zero()
        );
    }
}
