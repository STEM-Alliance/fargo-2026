package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.utils.FieldUtils;

public final class ShooterControlCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final Supplier<Pose2d> m_robotPoseSupplier;
    private final Supplier<ChassisSpeeds> m_robotSpeedsSupplier;

    public ShooterControlCommand(
        ShooterSubsystem shooter,
        Supplier<Pose2d> robotPoseSupplier,
        Supplier<ChassisSpeeds> robotSpeedsSupplier
    ) {
        m_shooter = shooter;
        m_robotPoseSupplier = robotPoseSupplier;
        m_robotSpeedsSupplier = robotSpeedsSupplier;

        addRequirements(m_shooter);
    }

    @Override
    public final void initialize() {}

    @Override
    public final void execute() {
        ShotCalculator.update(m_robotPoseSupplier.get(), m_robotSpeedsSupplier.get());

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
