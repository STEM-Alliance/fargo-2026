package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.utils.ShooterUtils;

public final class ShooterControlCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final IndexerSubsystem m_indexer;
    private final Supplier<Pose2d> m_robotPoseSupplier;
    private final Supplier<ChassisSpeeds> m_robotSpeedsSupplier;
    private final Debouncer m_autoShootDebouncer = new Debouncer(0.5, DebounceType.kRising);

    private boolean m_autoShootEnabled = false;

    public ShooterControlCommand(
        ShooterSubsystem shooter,
        IndexerSubsystem indexer,
        Supplier<Pose2d> robotPoseSupplier,
        Supplier<ChassisSpeeds> robotSpeedsSupplier,
        Trigger toggleAutoShootTrigger
    ) {
        m_shooter = shooter;
        m_indexer = indexer;
        m_robotPoseSupplier = robotPoseSupplier;
        m_robotSpeedsSupplier = robotSpeedsSupplier;

        toggleAutoShootTrigger.onTrue(Commands.runOnce(() -> {
            m_autoShootEnabled = !m_autoShootEnabled;
            SmartDashboard.putBoolean("AutoShootEnabled", m_autoShootEnabled);
        }));

        SmartDashboard.putNumber("HoodTargetAngle", 63.0);
        SmartDashboard.putBoolean("AutoShootEnabled", m_autoShootEnabled);

        addRequirements(m_shooter, m_shooter.getTurret());
    }

    @Override
    public final void initialize() {
        new Trigger(() -> m_autoShootEnabled && isScheduled() && ShotCalculator.shouldStartShooting() &&
            m_autoShootDebouncer.calculate(ShotCalculator.isShotPossible(m_robotPoseSupplier.get())
        )).whileTrue(Commands.parallel(
            // TODO: get shooting velocity like in new shotcalculator
            Commands.runOnce(() -> m_shooter.setFlywheelVelocity(null)),
            Commands.sequence(
                // TODO: Check velocity with the same timeout.
                Commands.waitSeconds(0.925),
                Commands.runOnce(() -> {
                    m_indexer.setRunning(true, false, false);
                    m_shooter.setKickerRunning(true, false);
                })
            )
        ).finallyDo(() -> {
            // We always stop indexing to stop shooting
            m_indexer.setRunning(false, false, false);

            // If we stopped because our shift ended, then we spin down the flywheel
            // We leave the kicker running as well to avoid jamming the shooter.
            if (!ShotCalculator.shouldStartShooting() || !m_autoShootEnabled || !isScheduled()) {
                m_shooter.setFlywheelVelocity(RadiansPerSecond.zero());
                m_shooter.setKickerRunning(false, false);
            }
        }));
    }

    @Override
    public final void execute() {
        ShotCalculator.update(
            m_robotPoseSupplier.get(),
            m_robotSpeedsSupplier.get()
        );

        Angle turretAzimuth = Degrees.of(
            ShotCalculator.getTargetTurretRelative().getAngle().unaryMinus()
            .plus(m_robotPoseSupplier.get().getRotation()).getDegrees()
        );

        Angle hoodElevation;
        if (!m_shooter.isFlywheelRunning()) {
            hoodElevation = Degrees.of(86.0);
        } else {
            hoodElevation = ShooterUtils.getTableAngle(Meters.of(ShotCalculator.getTargetTurretRelative().getNorm()));
        }

        m_shooter.setTurretAzimuth(turretAzimuth);
        m_shooter.setHoodElevation(hoodElevation);
    }

    @Override
    public final void end(boolean interrupted) {}

    @Override
    public final boolean isFinished() {
        return false;
    }
}
