package org.firstinspires.ftc.teamcode.math_utils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.SplineConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * Math for Lerp Path Planning
 */
public class LerpPathPlanning implements SplineConstants {
    private final Drivetrain drivetrain;
    private Point robotPosition;
    private final LerpPath[] paths;
    private LerpPath currentPath;
    private int index;

    /**
     * Instantiates the LerpPathPlanning
     *
     * @param drivetrain the drivetrain
     * @param paths all the paths we will follow
     */
    public LerpPathPlanning(Drivetrain drivetrain, LerpPath[] paths) {
        this.drivetrain = drivetrain;
        this.paths = paths;
        this.currentPath = this.paths[0];
    }

    /**
     * Loads the next path to follow
     */
    public void loadNextPath() {
        index++;
        if(index < paths.length)
            currentPath = paths[index];
    }


    private double lineProjection(double x) {
        return currentPath.slope * (x - currentPath.waypoint.x) + currentPath.waypoint.y;
    }

    private double optimalX() {
        return (robotPosition.x / currentPath.slope +
                currentPath.waypoint.x * currentPath.slope
                + robotPosition.y - currentPath.waypoint.y) /
                (currentPath.slope + 1.0 / currentPath.slope);
    }

    private double distanceOnLine() {
        double optimalX = optimalX();
        return Math.hypot(
                currentPath.waypoint.x - optimalX, currentPath.waypoint.y - lineProjection(optimalX));
    }

    private double fromLine() {
        double optimalX = optimalX();
        return Math.hypot(robotPosition.x - optimalX, robotPosition.y - lineProjection(optimalX));
    }

    private double desiredT(){
        return 1.0 - fromLine() / distanceOnLine();
    }

    private double projectedDistance() {
        double optimalX = optimalX();
        double optimalY = lineProjection(optimalX);
        double desiredT = desiredT();

        return Math.hypot(
                desiredT * (currentPath.waypoint.x - optimalX) +
                        optimalX - currentPath.waypoint.x,
                desiredT * (currentPath.waypoint.y - optimalY) +
                        optimalY - currentPath.waypoint.y);
    }

    /**
     * With the robot at (x, y), calculates the drive vector of the robot
     * in order to reach the waypoint
     *
     * @param turn the turn speed proportion
     * @param autoAlign whether to autoAlign
     * @param lowGear whether to drive in low gear
     *
     * @return if we are close enough to the destination area
     */
    public boolean spline(double turn, boolean autoAlign, boolean lowGear) {
        drivetrain.update();
        robotPosition = drivetrain.getRobotPose().toPoint();

        Vector driveVector;
        if(distanceOnLine() > projectedDistance()) {
            double optimalX = optimalX();
            double optimalY = lineProjection(optimalX);
            double desiredT = desiredT();

            driveVector = new Vector(
                    desiredT * (currentPath.waypoint.x - optimalX) + optimalX - robotPosition.x,
                    desiredT * (currentPath.waypoint.y - optimalY) + optimalY - robotPosition.y);
        }
        else {
            driveVector = new Vector(
                    optimalX() - robotPosition.x, lineProjection(optimalX()) - robotPosition.y);
        }

        double error = robotPosition.distance(currentPath.waypoint);

        boolean isFinished;
        if(error >= DESTINATION_ERROR) {
            driveVector.scaleMagnitude(SPLINE_P * error);
            isFinished = false;
        }
        else {
            driveVector.zero();
            isFinished = true;
        }

        drivetrain.drive(driveVector, turn, autoAlign, lowGear);
        return isFinished;
    }
}