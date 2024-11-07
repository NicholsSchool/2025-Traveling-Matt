package org.firstinspires.ftc.teamcode.math_utils;

import androidx.annotation.NonNull;

import java.util.Locale;

/**
 * A Vector in (x, y) notation
 */
public class Vector {
    /** X value */
    public double x;

    /** Y value */
    public double y;

    /**
     * Instantiates the Vector
     *
     * @param x the x value
     * @param y the y value
     */
    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * The absolute value of the Vector
     *
     * @return the distance from (0, 0)
     */
    public double magnitude() {
        return Math.hypot(x, y);
    }

    /**
     * The angle of the Vector
     *
     * @return the angle from (0, 0) in radians
     */
    public double angle() {
        return Math.atan2(y, x);
    }

    /**
     * Clips the magnitude of the Vector to the maximum
     * if it is above the maximum
     *
     * @param maxMagnitude the maximum Vector magnitude
     *
     * @return this vector after the value restriction
     */
    public Vector clipMagnitude(double maxMagnitude) {
        double magnitude = magnitude();
        if(magnitude > maxMagnitude) {
            double ratio = maxMagnitude / magnitude;
            x *= ratio;
            y *= ratio;
        }
        return this;
    }

    /**
     * Scales this Vector's magnitude to the new magnitude,
     * unless this Vector's magnitude is 0
     *
     * @param newMagnitude the new magnitude
     */
    public void scaleMagnitude(double newMagnitude) {
        double magnitude = magnitude();
        if(magnitude != 0) {
            double ratio = newMagnitude / magnitude;
            x *= ratio;
            y *= ratio;
        }
    }

    /**
     * Sets the vector to (0, 0)
     */
    public void zero() {
        x = 0.0;
        y = 0.0;
    }

    /**
     * The distance between two Vectors
     *
     * @param otherVector the Vector to compare to
     * @return the distance
     */
    public double distance(Vector otherVector) {
        return Math.hypot(x - otherVector.x, y - otherVector.y);
    }

    /**
     * Returns the Vector data as a String to three decimal places
     *
     * @return (x, y)
     */
    @NonNull
    public String toString() {
        return String.format(Locale.US, "(%.3f, %.3f)", x, y);
    }
}