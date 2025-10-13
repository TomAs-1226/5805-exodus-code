package frc.robot.util;

/**
 * terribly named math class
 * @author florpy
 */
public final class RobotMath {

    public static final double FLOATING_POINT_PRECISION = 10e-9;
    
    /**
     * dont use this (dont) ((ur not supposed to))
     */
    private RobotMath() {
        throw new UnsupportedOperationException("stop trying to instantiate util stuff");
    }

    /**
     * returns equality of two floating point numbers
     * @param number first number
     * @param otherNumber second number
     * @return number == otherNumber
     */
    public static boolean fEquals(double number, double otherNumber) {
        return Math.abs(number - otherNumber) <= FLOATING_POINT_PRECISION;
    }
   
    /**
     * clamps a number between two bounds
     * @param number the number to clamp
     * @param lowerBound the lower bound
     * @param upperBound the upper bound
     * @return number, less than or equal to upperBound and greater than or equal to lowerBound
     */
    public static double clamp(double number, double lowerBound, double upperBound) {
        return number < lowerBound ? lowerBound : number > upperBound ? upperBound : number;
    }

    /**
     * converts a 2-d vector to a degree
     * @param x x-coordinate
     * @param y y-coordinate
     * @return degree corresponding to the vector
     */
    public static double vecToDeg(double x, double y) {
        if (x == 0 && y == 0) return 0;
        double at2 = -(StrictMath.atan2(x, y) * (180/StrictMath.PI));
        at2 = at2 < 0 ? 360 + at2 : at2;
        return at2;
    }

    /**
     * returns the difference between the first angle and the second angle, in degrees
     * @param first first angle
     * @param second second angle
     * @return difference, signed
     */
    public static double getSignedAngleDiff(double first, double second) {
        double diff = first - second;
        diff += 180;
        double err = diff < 0 ? 360 + diff : diff % 360;
        err -= 180;
        return err;
    }

    /**
     * returns the difference between the first angle and the second angle, in degrees
     * @param first first angle
     * @param second second angle
     * @return difference, absolute
     */
    public static double getAngleAbsDiff(double first, double second) {
        return Math.abs(getSignedAngleDiff(first, second));
    }

    /**
     * not going to bother explaining
     * @param values all the comma values
     * @return max of all the values
     */
    public static double maxOf(double... values) {
        double max = Math.max(values[0], values[1]);
        for (int i = 2; i < values.length; i++) {
            max = Math.max(max, values[i]);
        }
        return max;
    }

    public static double deadzone(double n) {
        return Math.abs(n) < 0.075 ? 0 : n;
    }

}
