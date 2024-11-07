package org.firstinspires.ftc.teamcode.math_utils;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Restricts the rate of change of a value
 */
public class MotionProfile {
    private final ElapsedTime timer;
    private final double speed;
    private final double maxMagnitude;
    private double value;

    /**
     * Instantiates the Motion Profile
     *
     * @param speed the maximum speed
     * @param maxMagnitude the maximum output value magnitude
     */
    public MotionProfile(double speed, double maxMagnitude) {
        timer = new ElapsedTime();
        this.speed = speed;
        this.maxMagnitude = maxMagnitude;
    }

    /**
     * Calculates and stores the restricted output value
     *
     * @param inputValue the input value
     *
     * @return the restricted output value
     */
    public double calculate(double inputValue) {
        inputValue = Range.clip(inputValue, -maxMagnitude, maxMagnitude);

        double timePassed = timer.time();
        timer.reset();

        double deltaValue = inputValue - value;
        double maxDelta = timePassed * speed;

        if(deltaValue > maxDelta)
            value += maxDelta;
        else if(deltaValue < -maxDelta)
            value -= maxDelta;
        else
            value = inputValue;

        return value;
    }
}