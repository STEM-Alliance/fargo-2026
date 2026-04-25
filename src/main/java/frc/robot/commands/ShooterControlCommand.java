package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.utils.FieldUtils;
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
        SmartDashboard.putNumber("OffsetDegrees", 0.0);

        addRequirements(m_shooter, m_shooter.getTurret());
    }

    @Override
    public final void initialize() {
        // new Trigger(() -> m_autoShootEnabled && isScheduled() && ShotCalculator.shouldStartShooting() &&
        //     m_autoShootDebouncer.calculate(ShotCalculator.isShotPossible(m_robotPoseSupplier.get())
        // )).whileTrue(m_shooter.getShootCommand(m_indexer));
    }

    @Override
    public final void execute() {
        ShotCalculator.update(
            m_robotPoseSupplier.get(),
            m_robotSpeedsSupplier.get()
        );

        // Pose2d bluePose = m_robotPoseSupplier.get();

        // if (!FieldUtils.isBlueAlliance()) {
        //     bluePose = new Pose2d(
        //         FieldUtils.kFieldLength.in(Meters) - bluePose.getX(),
        //         FieldUtils.kFieldWidth.in(Meters) - bluePose.getY(),
        //         bluePose.getRotation().plus(Rotation2d.kPi)
        //     );
        // }

        // double widthinterpolated = 4.0 * (bluePose.getY() - FieldUtils.kFieldLength.in(Meters) * 0.5);
        // double rotationinterpolated = 1.0 * Math.abs(bluePose.getRotation().getRadians() - Math.PI);

        Angle turretAzimuth = Degrees.of(
            ShotCalculator.getTargetTurretRelative().getAngle().unaryMinus()
            .plus(m_robotPoseSupplier.get().getRotation()).getDegrees() +
            // depending on field x, interpolate between 4 and -4 for min (0.0) and max
            // also for rotation from 180 away from dirver station
            SmartDashboard.getNumber("OffsetDegrees", 0.0)
            // widthinterpolated + rotationinterpolated
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
