package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConfiguration.TurretConfiguration.kTurretOffset;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

import frc.robot.utils.FieldUtils;
import frc.robot.utils.ShooterUtils;

public final class ShotCalculator {
    private static final int kMaxIterations = 10;
    private static final Distance kSolutionTolerance = Meters.of(0.01);

    private static final Distance kHubExpansion = Meters.of(0.75);
    private static final Distance kHubTolerance = Meters.of(1.25);
    private static final LinearVelocity kPassingVelocity = MetersPerSecond.of(9.0);
    private static final Angle kPassingAngle = Degrees.of(35.0);

    private static final Distance kVelocityOffset = Meters.of(-0.325);
    private static final Distance kAngleOffset = Meters.of(0.0);

    private static Translation2d m_targetTurretRelative;
    private static Translation2d m_targetRobotRelative;
    private static Translation2d m_targetFieldRelative;
    private static LinearVelocity m_fuelVelocity;
    private static Angle m_launchAngle;
    private static Time m_timeOfFlight;

    private ShotCalculator() {}

    public static void update(
        Pose2d robotPose,
        ChassisSpeeds robotSpeeds
    ) {
        boolean passing = false;//!FieldUtils.inFriendlyAllianceZone(robotPose);

        Translation2d turretOffset = getTurretOffset(robotPose);
        Translation2d targetOffset = getTargetOffset(robotPose, passing);
        Translation2d inducedSpeeds = getInducedSpeeds(robotPose, robotSpeeds, turretOffset);

        Translation2d leadedOffset = targetOffset.minus(turretOffset);

        // The idea is that the robot is stationary and the target is instead moving with the negative
        // of the robot's velocity components. For each iteration, the launch angle and time of flight
        // are estimated and the target offset is shifted by how far it (or the robot) would move.
        for (int i = 0; i < kMaxIterations; i++) {
            if (passing) {
                m_fuelVelocity = kPassingVelocity;
                m_launchAngle = kPassingAngle;
            } else {
                m_fuelVelocity = ShooterUtils.getOptimalVelocity(Meters.of(leadedOffset.getNorm()));

                // This might be wrong; we are only accounting for the shooter
                // velocity and ingoring the induced velocity with the polynomial.
                // Maybe instead of moving the target and robot we sum the speeds?
                m_launchAngle = ShooterUtils.getQuadraticAngles(
                    Meters.of(leadedOffset.getNorm()).plus(kAngleOffset),
                    // 72.0 is the height from the floor to the center of the hub funnel.
                    // 25.5 is the height frmo the floor to the top of the shooter flywheel.
                    Meters.of(Units.inchesToMeters(72.0) - Units.inchesToMeters(25.5)),
                    m_fuelVelocity
                ).getSecond();

                // After we update the angle for the "optimal" velocity, we apply our offset to correct any distance error.
                m_fuelVelocity = ShooterUtils.getOptimalVelocity(Meters.of(leadedOffset.getNorm()).plus(kVelocityOffset));
            }

            double v = m_fuelVelocity.in(MetersPerSecond);
            double launchAngle = m_launchAngle.in(Radians);

            //if ((0.0 <= launchAngle) && (launchAngle < (Math.PI / 2.0))) {
                double nextLeadTime = leadedOffset.getNorm() / (v * Math.cos(launchAngle));

                Translation2d nextLeadedOffset = targetOffset
                    .minus(turretOffset)
                    .minus(inducedSpeeds.times(nextLeadTime));

                double delta = nextLeadedOffset.getDistance(leadedOffset);
                leadedOffset = nextLeadedOffset;

                if ((i == (kMaxIterations - 1)) || (delta <= kSolutionTolerance.in(Meters))) {
                    m_timeOfFlight = Seconds.of(nextLeadTime);
                    Logger.recordOutput("ShotCalculator/Iterations", i + 1);

                    break;
                }
            //}
        }

        m_targetTurretRelative = leadedOffset;
        m_targetRobotRelative = m_targetTurretRelative.minus(turretOffset);
        m_targetFieldRelative = m_targetRobotRelative.plus(robotPose.getTranslation());

        Logger.recordOutput("ShotCalculator/TargetTurretRelative", m_targetTurretRelative);
        Logger.recordOutput("ShotCalculator/TargetRobotRelative", m_targetRobotRelative);
        Logger.recordOutput("ShotCalculator/TargetFieldRelative", m_targetFieldRelative);
        Logger.recordOutput("ShotCalculator/FuelVelocity", m_fuelVelocity);
        Logger.recordOutput("ShotCalculator/LaunchAngle", m_launchAngle);
        Logger.recordOutput("ShotCalculator/TimeOfFlight", m_timeOfFlight);
    }

    public static Translation2d getTargetTurretRelative() {
        return m_targetTurretRelative;
    }

    public static Translation2d getTargetRobotRelative() {
        return m_targetRobotRelative;
    }

    public static Translation2d getTargetFieldRelative() {
        return m_targetFieldRelative;
    }

    public static LinearVelocity getFuelVelocity() {
        return m_fuelVelocity;
    }

    public static Angle getLaunchAngle() {
        return m_launchAngle;
    }

    public static Time getTimeOfFlight() {
        return m_timeOfFlight;
    }

    private static Translation2d getTurretOffset(Pose2d robotPose) {
        return kTurretOffset.rotateBy(robotPose.getRotation());
    }

    private static Translation2d getTargetOffset(Pose2d robotPose, boolean passing) {
        Translation2d target;

        if (passing) {
            target = FieldUtils.getPassingTarget(robotPose, kHubExpansion, kHubTolerance);
        } else {
            target = FieldUtils.getAllianceHub();
        }

        return target.minus(robotPose.getTranslation());
    }

    private static Translation2d getInducedSpeeds(
        Pose2d robotPose, 
        ChassisSpeeds robotSpeeds,
        Translation2d turretOffset
    ) {
        Translation2d linearSpeeds = new Translation2d(
            robotSpeeds.vxMetersPerSecond,
            robotSpeeds.vyMetersPerSecond
        ).rotateBy(robotPose.getRotation());

        // This does not also account for the rotation of the turret,
        // but that should be less significant than the robot rotation.
        Translation2d tangentialSpeeds = new Translation2d(
            -robotSpeeds.omegaRadiansPerSecond * turretOffset.getY(),
            robotSpeeds.omegaRadiansPerSecond * turretOffset.getX()
        );

        return linearSpeeds.plus(tangentialSpeeds);
    }
}
