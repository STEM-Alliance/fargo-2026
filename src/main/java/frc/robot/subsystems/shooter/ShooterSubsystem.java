package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.io.FlywheelIO;
import frc.robot.subsystems.shooter.kicker.Kicker;
import frc.robot.subsystems.shooter.kicker.io.KickerIO;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.io.TurretIO;

public final class ShooterSubsystem implements Subsystem {
    private final Kicker m_kicker;
    private final Turret m_turret;
    private final Flywheel m_flywheel;

    private boolean m_isShooting;

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

    public final boolean isShooting() {
        return m_isShooting;
    }

    public final void setKickerRunning(boolean running) {
        m_kicker.setRunning(running);
    }

    public final void setTurretAzimuth(Angle azimuth) {
        m_turret.setTurretAzimuth(azimuth);
    }

    public final void setHoodElevation(Angle elevation) {
        elevation = Degrees.of(Math.max(Math.min(elevation.in(Degrees), 67.0), 40.0));

        m_turret.setHoodAngle(elevation);
    }

    public final void setFlywheelVelocity(AngularVelocity motorVelocity) {
        m_isShooting = motorVelocity.abs(RotationsPerSecond) > 1e-6;
        m_flywheel.setMotorVelocities(motorVelocity);
    }

    public final void stopFlywheel() {
        m_isShooting = false;
        m_flywheel.setMotorVoltages(Volts.zero());
    }

    public final Flywheel getFlywheel() {
        return m_flywheel;
    }

    public final Command getTurretZeroRoutine() {
        Command turretZeroRoutine = Commands.parallel(
            m_turret.getHoodZeroRoutine(),
            m_turret.getTurretZeroRoutine()
        );

        turretZeroRoutine.addRequirements(this);

        return turretZeroRoutine.withName("TurretZeroRoutine");
    }
}
