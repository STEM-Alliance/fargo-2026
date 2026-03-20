package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConfiguration.TurretConfiguration.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.shooter.turret.io.TurretIO;
import frc.robot.subsystems.shooter.turret.io.TurretInputsAutoLogged;

public final class Turret {
    private final TurretIO m_turretIO;
    private final TurretInputsAutoLogged m_turretInputs;

    public Turret(TurretIO turretIO) {
        m_turretIO = turretIO;
        m_turretInputs = new TurretInputsAutoLogged();
    }

    public final void periodic() {
        m_turretIO.updateInputs(m_turretInputs);
        Logger.processInputs("ShooterSubsystem/Turret", m_turretInputs);
    }

    public final void setTurretAzimuth(Angle azimuth) {
        // We want to be able to wrap this to +-270.
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

    public final void stopHoodMotor() {
        m_turretIO.setHoodMotorVoltage(Volts.zero());
    }

    public final void setHoodVoltage(Voltage voltage) {
        m_turretIO.setHoodMotorVoltage(voltage);
    }

    public final void syncTurretMotorEncoder() {
        Angle rotations = m_turretInputs.turretEncoderPosition;
        Angle offset = kTurretEncoderZero.minus(rotations);

        m_turretIO.setTurretMotorPosition(offset.times(kTurretMotorRatio));
    }

    public final Command getTurretZeroRoutine() {
        return Commands.none();
        // return Commands.waitUntil(() -> m_turretInputs.isTurretEncoderConnected).withTimeout(Seconds.of(5.0))
        //     .andThen(Commands.runOnce(() -> {
        //         if (m_turretInputs.isTurretEncoderConnected) {
        //             syncTurretMotorEncoder();
        //         } else {
        //             m_turretIO.setTurretMotorPosition(Rotations.zero());
        //         }
        //     }));
        // return Commands.waitUntil(() -> m_turretInputs.isTurretEncoderConnected)
        //     .andThen(Commands.runOnce(this::syncTurretMotorEncoder));
    }

    public final Command getHoodZeroRoutine() {
        return Commands.sequence(
            Commands.startEnd(
                () -> m_turretIO.setHoodMotorVoltage(Volts.of(0.75)),
                () -> m_turretIO.setHoodMotorVoltage(Volts.zero())
            ).until(() -> m_turretInputs.hoodMotorStatorCurrent.abs(Amps) >= 7.5),

            Commands.waitUntil(() -> m_turretInputs.hoodMotorVelocity.abs(RotationsPerSecond) < 1e-6),
            Commands.runOnce(() -> m_turretIO.setHoodMotorPosition(kHoodMotorZero))
        );
    }
}
