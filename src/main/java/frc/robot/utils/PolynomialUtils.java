package frc.robot.utils;

public final class PolynomialUtils {
    private PolynomialUtils() {}

    /**
     * Evaluates a univariate polynomial at the given independent variable.
     * <p> Coefficients should be ordered in ascending degrees of x, with constants first.
     * 
     * @param coeffs The polynomial's coefficients.
     * @param x The independent variable.
     * @return The evaluated result.
    */
    public static double evaluateUnivariate(double[] coeffs, double x) {
        double result = 0.0;

        for (int i = coeffs.length - 1; i >= 0; i--) {
            result = Math.fma(result, x, coeffs[i]);
        }

        return result;
    }

    /**
     * Evaluates a bivariate polynomial at the given independent variables.
     * <p> Coefficients should be ordered in ascending degrees of x, then y, with constants first.
     * 
     * @param coeffs The polynomial's coefficients.
     * @param x The first independent variable.
     * @param y The second independent variable.
     * @return The evaluated result.
    */
    public static double evaluateBivariate(double[][] coeffs, double x, double y) {
        double result = 0.0;

        for (int i = coeffs.length - 1; i >= 0; i--) {
            result = Math.fma(result, x, evaluateUnivariate(coeffs[i], y));
        }

        return result;
    }
}
