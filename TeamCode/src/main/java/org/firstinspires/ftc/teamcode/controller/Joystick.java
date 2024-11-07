package org.firstinspires.ftc.teamcode.controller;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.math_utils.Vector;

import java.util.Locale;

/**
 * A Controller Joystick
 */
public class Joystick {
    /** X value */
    public Axis x;

    /** Y value */
    public Axis y;

    /**
     * Instantiates the Joystick
     */
    public Joystick() {
        x = new Axis();
        y = new Axis();
    }

    /**
     * The Joystick Position
     *
     * @return the position as a Vector
     */
    public Vector toVector() {
        return new Vector(x.value(), y.value());
    }

    /**
     * Updates the Joystick Axes
     *
     * @param newX the new X value
     * @param newY the new Y value
     */
    public void update(double newX, double newY) {
        x.update(newX);
        y.update(newY);
    }

    /**
     * Returns the Joystick data as a String to three decimal places
     *
     * @return (x, y)
     */
    @NonNull
    public String toString() {
        return String.format(Locale.US, "(%.3f, %.3f)", x.value(), y.value());
    }
}