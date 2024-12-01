package org.firstinspires.ftc.teamcode.math_utils;

import org.firstinspires.ftc.teamcode.constants.SplineConstants;

/**
 * A Lerp Path is defined by a Waypoint and a control point
 */
public class LerpPath implements SplineConstants {
    /**
     * The target waypoint
     */
    public final Point waypoint;

    /**
     * The angle to arrive at the waypoint at in radians
     */
    public double angle;

    /**
     * The control point defined by the slope and waypoint
     */
    public Point slopePoint;

    /**
     * The slope of the line formed by the waypoint and slope point
     */
    public double slope;

    /**
     * Instantiates the LerpPath
     *
     * @param waypoint the target waypoint
     * @param angle the target arrival angle
     */
    public LerpPath(Point waypoint, double angle) {
        this.waypoint = waypoint;
        this.angle = angle;

        double absAngle = Math.abs(Angles.clipRadians(this.angle));
        if(absAngle == Angles.PI_OVER_TWO || absAngle == Math.PI || absAngle == 0.0)
            this.angle += 0.0001;

        slope = Math.tan(this.angle);

        slopePoint = new Point(
                waypoint.x + Math.cos(this.angle),
                waypoint.y + Math.sin(this.angle));
    }
}
