package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConfiguration.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public final Command zeroHoodRoutine() {
        return Commands.sequence(
            Commands.runOnce(() -> m_turretIO.setHoodMotorVoltage(Volts.of(-6.0))),
            Commands.waitSeconds(0.05),
            Commands.runOnce(() -> m_turretIO.setHoodMotorVoltage(Volts.of(0.75))), // 2.0
            Commands.waitSeconds(0.1),
            Commands.waitUntil(() -> Math.abs(m_turretInputs.hoodMotorStatorCurrent.in(Amps)) > 7.5)
        ).finallyDo(() -> {
            m_turretIO.setHoodMotorVoltage(Volts.of(0.0));
            m_turretIO.setHoodMotorPosition(Radians.of(62.0 * 11.838));
            //m_turretIO.setHoodAngle(Degrees.of(61.0));
            // at 0 is 62, 22 at full.
            // 62 at 0, 22 at full
        });
        // return Commands.runEnd(
        //     () -> m_turretIO.setHoodMotorVoltage(Volts.of(1.0)),

        //     () -> {
        //         m_turretIO.setHoodMotorPosition(Degrees.of(90.0));
        //         m_turretIO.setHoodAngle(Degrees.of(90.0));
        //         m_turretIO.setHoodMotorVoltage(Volts.of(0.0));
        //     }
        // ).until(() -> Math.abs(m_turretInputs.hoodMotorStatorCurrent.in(Amps)) > 5.0);
    }

    public final void syncTurretMotorEncoder() {
        Angle rotations = m_turretInputs.turretEncoderPosition;
        double offset = kTurretEncoderZero.minus(rotations).in(Rotations);
        Angle currentPosition = Rotations.of(offset * kTurretMotorRatio);

        m_turretIO.setTurretMotorPosition(currentPosition);
    }

    public final void setTurretAzimuth(Angle azimuth) {
        double azimuthDegrees = azimuth.in(Degrees);

        if (Math.abs(azimuthDegrees) > 190.0) {
            if (azimuthDegrees >= 0.0) {
                azimuth = Degrees.of(azimuthDegrees - 360.0);
            } else {
                azimuth = Degrees.of(360.0 - azimuthDegrees);
            }
        }

        m_turretIO.setTurretAzimuth(azimuth);
    }

    public final void setHoodAngle(Angle angle) {
        m_turretIO.setHoodAngle(angle);
    }

    public final void setHoodMotorVoltage(Voltage voltage) {
        m_turretIO.setHoodMotorVoltage(voltage);
    }

    public final boolean isHomingSwitchPressed() {
        return m_turretInputs.isHoodHomingSwitchPressed;
    }
}
