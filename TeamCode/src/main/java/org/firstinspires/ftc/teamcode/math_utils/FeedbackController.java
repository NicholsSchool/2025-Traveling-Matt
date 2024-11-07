package org.firstinspires.ftc.teamcode.math_utils;

/**
 * A Feedback Controller designed to work against gravity
 */
public class FeedbackController {
    private final double proportional;
    private final double verticalProportional;
    private final double cosineCoefficient;
    private double targetPosition;

    /**
     * Instantiates the FeedbackController
     *
     * @param p the proportional constant
     * @param target the initial target position
     * @param v the power constant based on vertical error
     * @param verticalPosition the vertical position
     */
    public FeedbackController(double p, double target, double v, double verticalPosition) {
        proportional = p;
        targetPosition = target;
        verticalProportional = v;
        cosineCoefficient = Math.PI / (2.0 * verticalPosition);
    }

    /**
     * Sets the new Target Position
     *
     * @param newPosition the new position
     */
    public void setTargetPosition(double newPosition) {
        targetPosition = newPosition;
    }

    /**
     * Calculates the output value based on the error
     *
     * @param position the current position
     *
     * @return the output value
     */
    public double calculate(double position) {
        return proportional * (targetPosition - position) +
                verticalProportional * Math.cos(position * cosineCoefficient);
    }
}