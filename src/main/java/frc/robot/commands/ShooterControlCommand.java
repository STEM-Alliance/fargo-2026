package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.utils.FieldUtils;

public final class ShooterControlCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final Supplier<Pose2d> m_robotPoseSupplier;
    private final Supplier<ChassisSpeeds> m_robotSpeedsSupplier;

    private boolean m_autoShootEnabled = false;

    public ShooterControlCommand(
        ShooterSubsystem shooter,
        Supplier<Pose2d> robotPoseSupplier,
        Supplier<ChassisSpeeds> robotSpeedsSupplier,
        Trigger toggleAutoShootTrigger
    ) {
        m_shooter = shooter;
        m_robotPoseSupplier = robotPoseSupplier;
        m_robotSpeedsSupplier = robotSpeedsSupplier;

        toggleAutoShootTrigger.onTrue(Commands.runOnce(() -> {
            m_autoShootEnabled = !m_autoShootEnabled;
        }));

        addRequirements(m_shooter);
    }

    @Override
    public final void initialize() {}

    @Override
    public final void execute() {
        ShotCalculator.update(
            m_robotPoseSupplier.get(),
            m_robotSpeedsSupplier.get()
        );

        Angle turretAzimuth;
        if (FieldUtils.isBlueAlliance()) {
            turretAzimuth = Degrees.of(
                ShotCalculator.getTargetTurretRelative().getAngle().unaryMinus()
                .plus(m_robotPoseSupplier.get().getRotation()).getDegrees()
            );
        } else {
            turretAzimuth = Degrees.of(
                ShotCalculator.getTargetTurretRelative().getAngle().unaryMinus()
                .plus(m_robotPoseSupplier.get().getRotation())
                .plus(Rotation2d.fromDegrees(5.0)).getDegrees()
            );
        }

        // TODO: Move angle clamping and verification to the turret pseudo-subsystem.
        Angle hoodElevation = Degrees.of(
            MathUtil.clamp(ShotCalculator.getLaunchAngle().in(Degrees), 32.0, 67.0)
        );

        // since our hood can move so quickly, it should always be "stowed" unless we are trying to shoot.
        m_shooter.setTurretAzimuth(turretAzimuth);
        m_shooter.setHoodElevation(hoodElevation);
        //m_shooter.setHoodElevation(Degrees.of(SmartDashboard.getNumber("HoodElevation", 60.0)));
    }

    @Override
    public final void end(boolean interrupted) {}

    @Override
    public final boolean isFinished() {
        return false;
    }
}
