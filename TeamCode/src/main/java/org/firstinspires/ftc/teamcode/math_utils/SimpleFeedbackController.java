package org.firstinspires.ftc.teamcode.math_utils;

/**
 * A Simple Feedback Controller
 */
public class SimpleFeedbackController {
    private final double proportional;

    /**
     * Instantiates the SimpleFeedbackController
     *
     * @param p the proportional constant
     */
    public SimpleFeedbackController(double p) {
        proportional = p;
    }

    /**
     * Calculates the output value based on the error
     *
     * @param error the target position - the current position
     *
     * @return the output value
     */
    public double calculate(double error) {
        return proportional * error;
    }
}