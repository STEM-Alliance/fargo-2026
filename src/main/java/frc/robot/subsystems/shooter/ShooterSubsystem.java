package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.io.FlywheelIO;
import frc.robot.subsystems.shooter.kicker.io.KickerIO;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.io.TurretIO;

public final class ShooterSubsystem implements Subsystem {
    private final Turret m_turret;
    private final Flywheel m_flywheel;

    public ShooterSubsystem(
        KickerIO kickerIO,
        TurretIO turretIO,
        FlywheelIO flywheelIO
    ) {
        m_turret = new Turret(turretIO);
        m_flywheel = new Flywheel(flywheelIO);
    }

    @Override
    public final void periodic() {
        m_turret.periodic();
        m_flywheel.periodic();
    }

    public final void setHoodAngle(Angle angle) {
        m_turret.setHoodAngle(angle);
    }

    public final void stopHood() {
        m_turret.stopHoodMotor();
    }

    public final void setFlywheelVelocity(AngularVelocity motorVelocities) {
        m_flywheel.setMotorVelocities(motorVelocities);
    }

    public final void stopFlywheel() {
        m_flywheel.stopMotors();
    }

    public final Command getZeroRoutine() {
        // Parallel group commands share their requirements.
        Command shooterZeroRoutine = Commands.parallel(
            Commands.runOnce(m_turret::syncTurretMotorEncoder, this),
            m_turret.getHoodZeroRoutine()
        ).withName("ShooterZeroRoutine");

        return shooterZeroRoutine.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}
