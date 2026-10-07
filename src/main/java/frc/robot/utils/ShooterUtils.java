package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.*;

public final class ShooterUtils {
    private static final double[][] m_angleCoefficients = {
        { -594.1038163024,   477.6519866317,  -132.0205241202,    18.0418188371,    -1.2186535077,     0.0325429259},
        { -560.6245298939,   288.9881039029,   -57.0608638181,     5.0221927981,    -0.1651032380,     0.0000000000},
        { -131.1328958735,    52.9373601194,    -7.0868578949,     0.3135422073,     0.0000000000,     0.0000000000},
        {  -17.0362344199,     4.5686975341,    -0.3042728208,     0.0000000000,     0.0000000000,     0.0000000000},
        {   -1.1078890899,     0.1496821865,     0.0000000000,     0.0000000000,     0.0000000000,     0.0000000000},
        {   -0.0308469048,     0.0000000000,     0.0000000000,     0.0000000000,     0.0000000000,     0.0000000000}
    };

    private static final InterpolatingDoubleTreeMap m_hoodAngleTable = new InterpolatingDoubleTreeMap();

    static {
        m_hoodAngleTable.put(Double.MAX_VALUE, 63.0);
        m_hoodAngleTable.put(5.52308, 63.0);
        m_hoodAngleTable.put(3.7955, 68.5);
        m_hoodAngleTable.put(3.37, 71.5);
        m_hoodAngleTable.put(3.2038, 72.0);
        m_hoodAngleTable.put(2.76, 74.0);
        m_hoodAngleTable.put(2.45, 75.0);
        m_hoodAngleTable.put(1.9888, 77.5);
        m_hoodAngleTable.put(1.9023, 78.0);
        m_hoodAngleTable.put(1.0860, 84.0);
        m_hoodAngleTable.put(0.0, 84.0);
    }

    private static final double[] m_velocityCoefficients = {
        1.5324306535, 0.012682714, 0.0000120326
    };

    public static Angle getTableAngle(Distance distance) {
        return Degrees.of(m_hoodAngleTable.get(distance.in(Meters)));
    }

    public static Angle getPolynomialAngle(Distance distance, LinearVelocity velocity) {
        return Degrees.of(PolynomialUtils.evaluateBivariate(
            m_angleCoefficients,
            distance.in(Meters),
            velocity.in(MetersPerSecond)
        ));
    }

    public static LinearVelocity getPolynomialVelocity(AngularVelocity flywheelMotorVelocity) {
        return MetersPerSecond.of(PolynomialUtils.evaluateUnivariate(
            m_velocityCoefficients,
            flywheelMotorVelocity.in(RadiansPerSecond)
        ));
    }

    public static AngularVelocity getPolynomialVelocityRoot(LinearVelocity fuelVelocity) {
        double a = m_velocityCoefficients[2];
        double b = m_velocityCoefficients[1];
        double c = m_velocityCoefficients[0] - fuelVelocity.in(MetersPerSecond);

        return RadiansPerSecond.of(
            (-b + Math.sqrt(Math.pow(b, 2) - (4.0 * a * c))) / (2.0 * a)
        );
    }

    public static LinearVelocity getOptimalVelocity(Distance distance) {
        // Linear equation based off of simulation results, returns the
        // velocity in the middle of the "valley" of all possible shots.
        return MetersPerSecond.of(Math.min(distance.in(Meters) * 2.73 + 4.28, 12.0));
    }

    public static Pair<Angle, Angle> getQuadraticAngles(
        Distance horizontalDistance,
        Distance verticalDistance,
        LinearVelocity velocity
    ) {
        double v2 = Math.pow(velocity.in(MetersPerSecond), 2);
        double x = horizontalDistance.in(Meters);
        double y = verticalDistance.in(Meters);

        double principalRoot = Math.sqrt(Math.pow(v2, 2) - 9.8 * (9.8 * Math.pow(x, 2) + 2.0 * y * v2));
        double positiveRootAngle = Math.toDegrees(Math.atan((v2 + principalRoot) / (9.8 * x)));
        double negativeRootAngle = Math.toDegrees(Math.atan((v2 - principalRoot) / (9.8 * x)));

        return Pair.of(
            Degrees.of((positiveRootAngle < negativeRootAngle) ? positiveRootAngle : negativeRootAngle),
            Degrees.of((positiveRootAngle > negativeRootAngle) ? positiveRootAngle : negativeRootAngle)
        );
    }
}
