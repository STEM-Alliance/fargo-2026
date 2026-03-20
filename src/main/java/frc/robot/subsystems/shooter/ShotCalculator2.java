// package frc.robot.subsystems.shooter;

// import static edu.wpi.first.units.Units.*;
// import static frc.robot.subsystems.shooter.ShooterConfiguration.TurretConfiguration.kTurretOffset;

// import org.littletonrobotics.junction.Logger;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Twist2d;
// import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.units.measure.*;

// import frc.robot.utils.FieldUtils;
// import frc.robot.utils.ShooterUtils;

// public final class ShotCalculator2 {
//     private static final int kMaxIterations = 10;
//     private static final Distance kSolutionTolerance = Meters.of(0.01);

//     private static final double kLookAheadTime = 0.03;
//     private static final Angle kPassingAngle = Degrees.of(40.0);
//     private static final Distance kHubExpansion = Meters.of(0.75);
//     private static final Distance kHubTolerance = Meters.of(1.25);

//     private static Translation2d m_targetTurretRelative;
//     private static LinearVelocity m_fuelVelocity;
//     private static Angle m_launchAngle;
//     private static Time m_timeOfFlight;

//     private ShotCalculator2() {}

//     public static void update(
//         Pose2d robotPose,
//         ChassisSpeeds robotSpeeds
//     ) {
//         Pose2d futureRobotPose = getFutureRobotPose(robotPose, robotSpeeds, kLookAheadTime);

//         boolean isPassing = !FieldUtils.inFriendlyAllianceZone(futureRobotPose);
//         Translation2d inducedSpeeds = getInducedSpeeds(futureRobotPose, robotSpeeds);
//         Translation2d targetTurretOffset = getTargetTurretOffset(futureRobotPose, isPassing);

//         Translation2d leadedOffset = targetTurretOffset;
//         double fuelVelocity = 0.0;
//         double hoodAngle = 0.0;
//         double timeOfFlight = 0.0;

//         for (int i = 0; i < kMaxIterations; i++) {
//             double distance = leadedOffset.getNorm();

//             if (isPassing) {
//                 // I think what we want to do is set a fuelVelocity through our dashboard
//                 // and then step through values of the shooter velocity until it matches,
//                 // instead of trying to map actual shots to actual velocities. then those
//                 // values can be used instead of the camera measurements.
//                 fuelVelocity = ShooterUtils.getPassingVelocity(Meters.of(distance));
//                 hoodAngle = kPassingAngle.in(Degrees);
//             } else {
//                 // put the velocity on the dashboard to adjust, plus a shooter rpm
//                 // and then the distance. adjust velocity to reach a good angle and then get rpm to hit target.
//                 fuelVelocity = ShooterUtils.getShootingVelocity(Meters.of(distance));

//                 // v = sqrt(9.8 * (y + sqrt(x^2 + y^2)))

//                 hoodAngle = ShooterUtils.getQuadraticAngles(
//                     Meters.of(distance),
//                     Inches.of(71.5 - 22.5),
//                     MetersPerSecond.of(fuelVelocity)
//                 ).getSecond().in(Degrees);
//             }

//             timeOfFlight = distance / (fuelVelocity * Math.cos(Math.toRadians(hoodAngle)));
//             Translation2d nextLeadedOffset = targetTurretOffset.minus(inducedSpeeds.times(timeOfFlight));
//             double delta = nextLeadedOffset.getDistance(leadedOffset);

//             leadedOffset = nextLeadedOffset;

//             if ((i >= (kMaxIterations - 1)) || (delta <= kSolutionTolerance.in(Meters))) {
//                 Logger.recordOutput("ShotCalculator/Iterations", i + 1);
//                 break;
//             }
//         }

//         m_targetTurretRelative = leadedOffset;
//         m_fuelVelocity = MetersPerSecond.of(fuelVelocity);
//         m_launchAngle = Degrees.of(hoodAngle);
//         m_timeOfFlight = Seconds.of(timeOfFlight);

//         Logger.recordOutput("ShotCalculator/TargetTurretRelative", m_targetTurretRelative);
//         Logger.recordOutput("ShotCalculator/FuelVelocity", fuelVelocity);
//         Logger.recordOutput("ShotCalculator/LaunchAngle", m_launchAngle);
//         Logger.recordOutput("ShotCalculator/TimeOfFlight", m_timeOfFlight);
//     }

//     public static Translation2d getTargetTurretRelative() {
//         return m_targetTurretRelative;
//     }

//     public static LinearVelocity getFuelVelocity() {
//         return m_fuelVelocity;
//     }

//     public static Angle getLaunchAngle() {
//         return m_launchAngle;
//     }

//     public static Time getTimeOfFlight() {
//         return m_timeOfFlight;
//     }

//     private static Pose2d getFutureRobotPose(
//         Pose2d robotPose,
//         ChassisSpeeds robotSpeeds,
//         double dtSeconds
//     ) {
//         return robotPose.exp(new Twist2d(
//             robotSpeeds.vxMetersPerSecond * dtSeconds,
//             robotSpeeds.vyMetersPerSecond * dtSeconds,
//             robotSpeeds.omegaRadiansPerSecond * dtSeconds
//         ));
//     }

//     private static Translation2d getTargetTurretOffset(Pose2d robotPose, boolean isPassing) {
//         Translation2d turretRobotOffset = kTurretOffset.rotateBy(robotPose.getRotation());
//         Translation2d targetFieldOffset;

//         if (isPassing) {
//             targetFieldOffset = FieldUtils.getPassingTarget(robotPose, kHubExpansion, kHubTolerance);
//         } else {
//             targetFieldOffset = FieldUtils.getAllianceHub();
//         }

//         return targetFieldOffset.minus(robotPose.getTranslation().plus(turretRobotOffset));
//     }

//     private static Translation2d getInducedSpeeds(Pose2d robotPose, ChassisSpeeds robotSpeeds) {
//         Translation2d turretRobotOffset = kTurretOffset.rotateBy(robotPose.getRotation());

//         Translation2d linearSpeeds = new Translation2d(
//             robotSpeeds.vxMetersPerSecond,
//             robotSpeeds.vyMetersPerSecond
//         ).rotateBy(robotPose.getRotation());

//         Translation2d tangentialSpeeds = new Translation2d(
//             -robotSpeeds.omegaRadiansPerSecond * turretRobotOffset.getY(),
//             robotSpeeds.omegaRadiansPerSecond * turretRobotOffset.getX()
//         );

//         return linearSpeeds.plus(tangentialSpeeds);
//     }
// }
