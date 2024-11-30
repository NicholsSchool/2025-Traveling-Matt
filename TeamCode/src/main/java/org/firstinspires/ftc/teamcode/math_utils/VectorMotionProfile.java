package org.firstinspires.ftc.teamcode.math_utils;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Restricts the rate of change of a Vector on an (x, y) plane
 */
public class VectorMotionProfile {
    private final ElapsedTime timer;
    private final Vector vector;
    private final double speed;

    /**
     * Instantiates the Vector Motion Profile
     *
     * @param speed the maximum speed
     */
    public VectorMotionProfile(double speed) {
        timer = new ElapsedTime();
        vector = new Vector(0.0, 0.0);
        this.speed = speed;
    }

    /**
     * Calculates and stores the restricted output value
     *
     * @param inputVector the input Vector
     *
     * @return the restricted output Vector
     */
    public Vector calculate(Vector inputVector) {
        double timePassed = timer.time();
        timer.reset();

        double delta = vector.distance(inputVector);
        double maxDelta = speed * timePassed;

        if(delta <= maxDelta) {
            vector.x = inputVector.x;
            vector.y = inputVector.y;
        }
        else {
            double ratio = maxDelta / delta;
            vector.x += (inputVector.x - vector.x) * ratio;
            vector.y += (inputVector.y - vector.y) * ratio;
        }

        return vector;
    }
}