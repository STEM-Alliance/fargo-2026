package frc.robot.subsystems.shooter.kicker;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.shooter.kicker.io.KickerIO;
import frc.robot.subsystems.shooter.kicker.io.KickerInputsAutoLogged;

public final class Kicker {
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

    public final void start() {
        m_kickerIO.setKickerMotorVoltage(Volts.of(12.0));
    }

    public final void stop() {
        m_kickerIO.setKickerMotorVoltage(Volts.zero());
    }
}
