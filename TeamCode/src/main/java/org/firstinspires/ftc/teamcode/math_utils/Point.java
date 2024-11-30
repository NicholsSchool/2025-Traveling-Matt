package org.firstinspires.ftc.teamcode.math_utils;

import androidx.annotation.NonNull;

import java.util.Locale;

/**
 * A Point (x, y)
 */
public class Point {
    /**
     * X value
     */
    public double x;

    /**
     * Y value
     */
    public double y;

    /**
     * Instantiates the Point
     *
     * @param x the x value
     * @param y the y value
     */
    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * The distance between two Points
     *
     * @param otherPoint the Point to compare to
     * @return the distance
     */
    public double distance(Point otherPoint) {
        return Math.hypot(x - otherPoint.x, y - otherPoint.y);
    }

    /**
     * The unscaled slope (explicit point - implicit point)
     *
     * @param otherPoint the explicit Point to compare to
     * @return the slope as an unscaled (x, y) vector
     */
    public Vector slope(Point otherPoint) {
        return new Vector(otherPoint.x - x, otherPoint.y - y);
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