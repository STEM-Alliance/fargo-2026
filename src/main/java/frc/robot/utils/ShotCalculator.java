package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.*;

public final class ShotCalculator {
    // Turret offset is relative to the robot's front, WPILib coordinates.
    private static final Translation2d kTurretOffset = new Translation2d(0.0, 0.0);
    private static final Distance kTurretHeight = Meters.of(0.762);
    private static final Distance kFuelDiameter = Meters.of(0.150);
    private static final Distance kHubHeightDifference = Meters.of(1.58);
    private static final LinearVelocity kFuelVelocity = MetersPerSecond.of(8.0);

    private static final Angle kPassingAngle = Degrees.of(22.5);
    private static final Distance kPassingHubExpansion = Meters.of(0.75);
    private static final Distance kPassingDirectTolerance = Meters.of(1.25);

    private Translation2d m_targetFieldTranslation;
    private Translation2d m_targetRobotTranslation;
    private Angle m_launchAngle;
    private Time m_timeOfFlight;

    public ShotCalculator() {}

    public final void updateForPassing(
        Pose2d robotPose,
        ChassisSpeeds robotSpeeds
    ) {
        update(
            robotPose,
            robotSpeeds,
            FieldUtils.getPassingTarget(
                robotPose,
                kPassingHubExpansion,
                kPassingDirectTolerance
            ),
            kFuelVelocity,
            kPassingAngle
        );
    }

    public final void updateForScoring(
        Pose2d robotPose,
        ChassisSpeeds robotSpeeds
    ) {
        update(
            robotPose,
            robotSpeeds,
            FieldUtils.getAllianceHub(),
            kFuelVelocity,
            null
        );
    }

    public final void update(
        Pose2d robotPose,
        ChassisSpeeds robotSpeeds,
        Translation2d targetTranslation,
        LinearVelocity projectileVelocity,
        Angle staticAngle
    ) {
        Translation2d turretTranslation = robotPose.getTranslation()
            .plus(kTurretOffset.rotateBy(robotPose.getRotation()));

        m_targetRobotTranslation = ShooterUtils.getLeadedTranslation(
            new Pose2d(turretTranslation, robotPose.getRotation()),
            targetTranslation,
            kFuelVelocity,
            robotSpeeds,
            staticAngle
        );

        m_targetFieldTranslation = m_targetRobotTranslation.plus(robotPose.getTranslation());

        if (staticAngle == null) {
            m_launchAngle = ShooterUtils.getQuadraticAngles(
                Meters.of(m_targetRobotTranslation.getNorm()),
                kHubHeightDifference,
                kFuelVelocity
            ).getSecond();
        } else {
            m_launchAngle = staticAngle;
        }

        m_timeOfFlight = Seconds.of(
            m_targetRobotTranslation.getNorm() / (kFuelVelocity.in(MetersPerSecond) * Math.cos(m_launchAngle.in(Radians)))
        );
    }

    public final Translation2d getTargetFieldTranslation() {
        return m_targetFieldTranslation;
    }

    public final Translation2d getTargetRobotTranslation() {
        return m_targetRobotTranslation;
    }

    public final Angle getLaunchAngle() {
        return m_launchAngle;
    }

    public final Time getTimeOfFlight() {
        return m_timeOfFlight;
    }
}
