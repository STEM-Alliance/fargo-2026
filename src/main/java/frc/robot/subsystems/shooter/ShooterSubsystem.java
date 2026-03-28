package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.io.FlywheelIO;
import frc.robot.subsystems.shooter.kicker.Kicker;
import frc.robot.subsystems.shooter.kicker.io.KickerIO;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.io.TurretIO;
import frc.robot.utils.ShooterUtils;

public final class ShooterSubsystem implements Subsystem {
    private final Kicker m_kicker;
    private final Turret m_turret;
    private final Flywheel m_flywheel;

    private boolean m_isFlywheelRunning = false;

    public ShooterSubsystem(
        KickerIO kickerIO,
        TurretIO turretIO,
        FlywheelIO flywheelIO
    ) {
        m_kicker = new Kicker(kickerIO);
        m_turret = new Turret(turretIO);
        m_flywheel = new Flywheel(flywheelIO);
    }

    @Override
    public final void periodic() {
        m_kicker.periodic();
        m_turret.periodic();
        m_flywheel.periodic();
    }

    public final void setKickerRunning(boolean running, boolean reverse) {
        m_kicker.setRunning(running, reverse);
    }

    public final void setTurretAzimuth(Angle azimuth) {
        m_turret.setTurretAzimuth(azimuth);
    }

    public final void setHoodElevation(Angle elevation) {
        m_turret.setHoodAngle(elevation);
    }

    public final void setFlywheelVelocity(AngularVelocity motorVelocity) {
        m_isFlywheelRunning = motorVelocity.abs(RadiansPerSecond) > 1e-6;
        m_flywheel.setMotorVelocities(motorVelocity);
    }

    public final boolean isFlywheelRunning() {
        return m_isFlywheelRunning;
    }

    public final Command getShootCommand(IndexerSubsystem indexer) {
        return Commands.parallel(
            Commands.run(() -> {
                setFlywheelVelocity(ShooterUtils.getPolynomialVelocityRoot(
                    ShotCalculator.getFuelVelocity()
                ));
            }, m_flywheel),

            Commands.sequence(
                // TODO: Check velocity with timeout.
                Commands.waitSeconds(0.925),
                Commands.run(() -> {
                    indexer.setRunning(true, false);
                    setKickerRunning(true, false);
                }, indexer, m_kicker)
            ).finallyDo(() -> {
                setFlywheelVelocity(RadiansPerSecond.zero());
                setKickerRunning(false, false);
                indexer.setRunning(false, false);
            })
        );
    }

    public final Kicker getKicker() {
        return m_kicker;
    }

    public final Turret getTurret() {
        return m_turret;
    }

    public final Flywheel getFlywheel() {
        return m_flywheel;
    }
}
