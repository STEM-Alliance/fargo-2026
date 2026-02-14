package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.*;

import frc.robot.subsystems.shooter.turret.io.TurretIO;
import frc.robot.subsystems.shooter.turret.io.TurretIO.TurretInputs;
import frc.robot.subsystems.shooter.turret.io.TurretInputsAutoLogged;

/**
 * A self-contained turret pseudo-subsystem.
*/
public final class Turret {
    private final TurretIO m_turretIO;
    private final TurretInputsAutoLogged m_turretInputs = new TurretInputsAutoLogged();

    public Turret(TurretIO turretIO) {
        m_turretIO = turretIO;
    }

    public final void periodic() {
        m_turretIO.updateInputs(m_turretInputs);
        Logger.processInputs("ShooterSubsystem/Turret", m_turretInputs);
    }

    public final TurretInputs getInputs() {
        return m_turretInputs;
    }

    public final void setTurretAzimuth(Angle azimuth) {
        m_turretIO.setTurretAzimuth(azimuth);
    }

    public final void setHoodAngle(Angle angle) {
        m_turretIO.setHoodAngle(angle);
    }

    public final void resetTurretMotorPosition(Angle position) {
        m_turretIO.setTurretMotorPosition(position);
    }

    public final void resetHoodMotorPosition(Angle position) {
        m_turretIO.setHoodMotorPosition(position);
    }

    public final void setHoodMotorVoltage(Voltage voltage) {
        m_turretIO.setHoodMotorVoltage(voltage);
    }

    public final boolean isHomingSwitchPressed() {
        return m_turretInputs.isHoodHomingSwitchPressed;
    }
}
