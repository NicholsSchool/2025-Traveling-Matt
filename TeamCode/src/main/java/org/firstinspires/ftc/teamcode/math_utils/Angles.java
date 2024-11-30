package org.firstinspires.ftc.teamcode.math_utils;

/**
 * Angle Constants and Operations
 */
public class Angles {
    /** The ratio of a circle's circumference to its radius */
    public static final double TAO = 2.0 * Math.PI;

    /** Literally Pi divided by 2 */
    public static final double PI_OVER_TWO = Math.PI / 2.0;

    /** Literally -1 * Pi divided by 2 */
    public static final double NEGATIVE_PI_OVER_TWO = -Math.PI / 2.0;

    /**
     * Clips the angle in the range [-180, 180)
     *
     * @param angle the angle to clip
     *
     * @return the clipped angle
     */
    public static double clipDegrees(double angle) {
        while(angle >= 180.0)
            angle -= 360.0;
        while(angle < -180.0)
            angle += 360.0;
        return angle;
    }

    /**
     * Clips the angle in the range [-Pi, Pi)
     *
     * @param angle the angle to clip
     *
     * @return the clipped angle
     */
    public static double clipRadians(double angle) {
        while(angle >= Math.PI)
            angle -= TAO;
        while(angle < -Math.PI)
            angle += TAO;
        return angle;
    }

    /**
     * Averages two angles in radians, accounting for angle ranges
     *
     * @param angle1 the first angle in radians
     * @param angle2 the second angle in radians
     *
     * @return the average in radians
     */
    public static double average(double angle1, double angle2) {
        return Math.atan2(Math.sin(angle1) + Math.sin(angle2), Math.cos(angle1) + Math.cos(angle2));
    }
}