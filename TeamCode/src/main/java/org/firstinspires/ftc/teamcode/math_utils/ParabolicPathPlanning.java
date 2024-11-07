package org.firstinspires.ftc.teamcode.math_utils;

import org.firstinspires.ftc.teamcode.constants.SplineConstants;

/**
 * Math for Parabolic Path Planning
 */
public class ParabolicPathPlanning implements SplineConstants {
    /**
     * Instantiates the Parabolic Spline
     */
    public ParabolicPathPlanning() {}

    /**
     * With the robot at (rx, ry), calculates the drive vector of the robot
     * in order to follow a parabola and arrive at the waypoint (wx, wy)
     * that is the parabola's vertex.
     * The parabola is defined to contain the robot
     *
     * @param robot the robot (rx, ry)
     * @param waypoint the waypoint (wx, wy)
     * @param toIntake whether the robot is going to the intake
     *
     * @return the unscaled drive vector in [x, y] notation
     */
    public Vector vectorToVertex(Point robot, Point waypoint, boolean toIntake) {
        if(robot.x == waypoint.x)
            return toIntake ? new Vector(1.0, 0.0) : new Vector(-1.0, 0.0);

        Vector slope = robot.slope(waypoint);
        slope.y *= 2;
        return slope;
    }
}