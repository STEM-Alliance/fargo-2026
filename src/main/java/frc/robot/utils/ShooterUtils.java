package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public final class ShooterUtils {
    private static final double[][] m_angleCoefficients = {
        {  -71.266188447569920,   92.239379642263190,  -20.485516376989940,    2.209791809586092,   -0.115910911316219,    0.002368592334477},
        { -161.684430508642980,   60.667295795086860,   -8.891620351789737,    0.581145838107222,   -0.014099175681324,    0.000000000000000},
        {  -23.929486123536780,    7.360878175921579,   -0.737863730283898,    0.023983217832612,    0.000000000000000,    0.000000000000000},
        {   -2.364418630234339,    0.452911516474226,   -0.021204463188639,    0.000000000000000,    0.000000000000000,    0.000000000000000},
        {   -0.091830256743059,    0.008526541591285,    0.000000000000000,    0.000000000000000,    0.000000000000000,    0.000000000000000},
        {   -0.001521081562816,    0.000000000000000,    0.000000000000000,    0.000000000000000,    0.000000000000000,    0.000000000000000}
    };

    public static Angle getPolynomialAngle(Distance distance, LinearVelocity velocity) {
        return Degrees.of(PolynomialUtils.evaluateBivariate(
            m_angleCoefficients,
            distance.in(Meters),
            velocity.in(MetersPerSecond)
        ));
    }

    public static Pair<Angle, Angle> getQuadraticAngles(
        Distance horizontalDistance,
        Distance verticalDistance,
        LinearVelocity velocity
    ) {
        double v = velocity.in(MetersPerSecond);
        double x = horizontalDistance.in(Meters);
        double y = verticalDistance.in(Meters);

        double principal = Math.sqrt(Math.pow(v, 4) - 9.8 * (9.8 * Math.pow(x, 2) + 2.0 * y * Math.pow(v, 2)));
        Angle principalAngle = Degrees.of(Math.toDegrees(Math.atan((Math.pow(v, 2) + principal) / (9.8 * x))));
        Angle secondaryAngle = Degrees.of(Math.toDegrees(Math.atan((Math.pow(v, 2) - principal) / (9.8 * x))));

        return Pair.of(
            principalAngle.lt(secondaryAngle) ? principalAngle : secondaryAngle,
            principalAngle.gt(secondaryAngle) ? principalAngle : secondaryAngle
        );
    }

    public static Translation2d getLeadedTranslation(
        Pose2d robotPose,
        Translation2d targetTranslation,
        LinearVelocity projectileVelocity,
        ChassisSpeeds robotSpeeds,
        int iterations
    ) {
        Translation2d targetRelative = targetTranslation.minus(robotPose.getTranslation());
        ChassisSpeeds targetSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, robotPose.getRotation());
        Translation2d robotSpeedsVector = new Translation2d(
            targetSpeeds.vxMetersPerSecond,
            targetSpeeds.vyMetersPerSecond
        );

        double v = projectileVelocity.in(MetersPerSecond);
        Translation2d leadVector = targetRelative;

        for (int i = 0; i < iterations; i++) {
            double launchAngle = getQuadraticAngles(
                Meters.of(leadVector.getNorm()),
                Meters.of(1.58),
                projectileVelocity
            ).getSecond().in(Radians);

            if ((0.0 < launchAngle) && (launchAngle < (Math.PI / 2.0))) {
                double nextLeadTime = leadVector.getNorm() / (v * Math.cos(launchAngle));
                leadVector = targetRelative.minus(robotSpeedsVector.times(nextLeadTime));
            } else {
                return targetRelative;
            }
        }

        return leadVector;
    }

    public static Translation2d getLeadedTranslation(
        Pose2d robotPose,
        Translation2d targetTranslation,
        LinearVelocity projectileVelocity,
        ChassisSpeeds robotSpeeds
    ) {
        return getLeadedTranslation(
            robotPose,
            targetTranslation,
            projectileVelocity,
            robotSpeeds,
            5
        );
    }

    public static Translation2d getShooterTarget(
        Pose2d robotPose,
        ChassisSpeeds robotSpeeds,
        LinearVelocity projectileVelocity
    ) {
        // If we are in our alliance zone, we want to score. In all
        // other cases, we want to pass to our alliance zone (or to
        // our alliance's half of the field if pass isn't possible.
        boolean scoring = FieldUtils.inFriendlyAllianceZone(robotPose);

        if (scoring) {
            return getLeadedTranslation(
                robotPose,
                FieldUtils.getAllianceHub(),
                projectileVelocity,
                robotSpeeds
            );
        }

        Translation2d target = FieldUtils.getPassingTarget(
            robotPose,
            !FieldUtils.shotIntersectsHub(robotPose, 0.85) &&
             FieldUtils.inNeutralZone(robotPose)
        );

        return target;
    }
}
