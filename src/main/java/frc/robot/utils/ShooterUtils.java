package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.*;

public final class ShooterUtils {
    private static final double[][] m_angleCoefficients = {
        {  3389.486004565701477,  -397.528277605048572, -1075.503117627168421,    462.622280784542397,  -68.393732133847620,   3.342514698879313},
        { -2530.778725952642616,  4956.036810751237680, -2338.662316331670808,    403.643062621687022,   -22.40682636493625,   0.000000000000000},
        { -4250.044565640008841,  3497.284239981390783,  -827.460783399764068,     56.061603770760086,     0.00000000000000,   0.000000000000000},
        { -1621.282913177519503,   709.343592205059280,   -65.318515504337710,      0.000000000000000,     0.00000000000000,   0.000000000000000},
        {  -214.969655905166007,    34.829546035331929,     0.0000000000000000,     0.000000000000000,     0.00000000000000,   0.000000000000000},
        {    -6.489619195975671,     0.000000000000000,     0.0000000000000000,     0.000000000000000,     0.00000000000000,   0.000000000000000}
    };

    private static final double[] m_velocityCoefficients = {
        1.5324306535, 0.012682714, 0.0000120326
    };

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
        return MetersPerSecond.of(distance.in(Meters) * 0.7953 + 4.396);
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
