package math;

import java.util.List;

/**
 * Contains basic functions that are used often.
 */
public class Util {

    public static final double EPSILON = 1E-9;

    /** Prevent this class from being instantiated. */
    private Util() {
    }

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static String joinStrings(String delim, List<?> strings) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    public static boolean withinRange(double value, double min, double max) {
        return (value >= min) && (value <= max);
    }

    public static boolean withinDeadband(double value, double magnitude) {
        return withinRange(value, -magnitude, magnitude);
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return withinDeadband(a - b, epsilon);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, EPSILON);
    }

    public static boolean allCloseTo(List<Double> list, double value, double epsilon) {
        boolean result = true;
        for (Double value_in : list) {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }
}
