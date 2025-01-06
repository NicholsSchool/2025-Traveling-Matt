package org.firstinspires.ftc.teamcode.math_utils;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.components.LimelightComponent;
import org.firstinspires.ftc.teamcode.subsystems.components.OpticalSensor;

import java.util.Optional;

/**
 * The Robot Pose (x, y, theta)
 */
public class PoseEstimator {
    public Pose2D initialPose;

    public Pose2D robotPose;

    public LimelightComponent limelight;

    public OpticalSensor otos;

    public boolean useLL;

    public boolean isUsingLL;

    /**
     * The Field-Relative Robot Pose.
     * @param hwMap OpMode Hardware Map passthrough for LL, OTOS, and Gyro initialization.
     * @param initialPose Pose2D for robot's initial field-relative position.
     */
    public PoseEstimator(HardwareMap hwMap, Pose2D initialPose, boolean useLL) {
        this.useLL = useLL;
        Optional<Point> LLPose = null;
        if (useLL) {
            limelight = new LimelightComponent(hwMap);
            limelight.updateWithPose(initialPose.getHeading(AngleUnit.DEGREES));
            LLPose = limelight.getRobotPose();
        }

        otos = new OpticalSensor("OTOS", hwMap, DistanceUnit.METER, AngleUnit.DEGREES);

        //If the limelight can localize at startup, use that for the initial pose.
        if (useLL && LLPose.isPresent() ) {
            this.initialPose = new Pose2D(
                    DistanceUnit.METER,
                    LLPose.get().x,
                    LLPose.get().y,
                    AngleUnit.DEGREES,
                    initialPose.getHeading(AngleUnit.DEGREES)
            );
        } else {
            this.initialPose = new Pose2D(
                    DistanceUnit.METER,
                    initialPose.getX(DistanceUnit.METER),
                    initialPose.getY(DistanceUnit.METER),
                    AngleUnit.DEGREES,
                    initialPose.getHeading(AngleUnit.DEGREES)
            );

            this.robotPose = this.initialPose;

        }
    }

    public Pose2D getPose() { return robotPose; }

    public boolean isUsingLL() { return isUsingLL; }

    public void update() {
        otos.update();

        if (useLL) limelight.updateWithPose(robotPose.getHeading(AngleUnit.DEGREES));

        Optional<Point> LLEstimate = limelight.getRobotPose();

        isUsingLL = LLEstimate.isPresent();

        LLEstimate.ifPresent(point -> robotPose = new Pose2D(
                DistanceUnit.METER,
                point.x,
                point.y,
                AngleUnit.DEGREES,
                getFieldHeading(AngleUnit.DEGREES)));

        if( !LLEstimate.isPresent() ) {
            Vector transformedDeltas = transformFieldOriented(otos.getPosDeltas());
            robotPose = new Pose2D(
                    DistanceUnit.METER,
                    robotPose.getX(DistanceUnit.METER) + transformedDeltas.x,
                    robotPose.getY(DistanceUnit.METER) + transformedDeltas.y,
                    AngleUnit.DEGREES,
                    getFieldHeading(AngleUnit.DEGREES)
            );
        }
    }

    public double getInitialHeading(AngleUnit unit) {
        return initialPose.getHeading(unit);
    }

    private double getFieldHeading(AngleUnit unit) {
        if (unit == AngleUnit.DEGREES) {
            return Angles.clipDegrees(initialPose.getHeading(AngleUnit.DEGREES) + otos.getHeading());
        } else {
            return Angles.clipRadians(initialPose.getHeading(AngleUnit.RADIANS) + Math.toRadians(otos.getHeading()));
        }
    }

    /**
     * Takes in a vector that is robot-oriented (such as OTOS position/deltas) and converts it to
     * field-oriented using the field heading calculated from the gyro and inputted initialHeading.
     * @param inputVector The robot-oriented vector.
     * @return The field-oriented vector.
     */
    private Vector transformFieldOriented(Vector inputVector) {
        return new Vector(
                (Math.cos(getFieldHeading(AngleUnit.RADIANS)) * inputVector.x) - (Math.sin(getFieldHeading(AngleUnit.RADIANS)) * inputVector.y),
                (Math.sin(getFieldHeading(AngleUnit.RADIANS)) * inputVector.x) + (Math.cos(getFieldHeading(AngleUnit.RADIANS)) * inputVector.y)
        );
    }

    /**
     * Method for testing the robot to field transformation using the OTOS data (not deltas)
     * @return The transformed Vector.
     */
    public Pose2D debugTransform() {
        Vector otosPos = otos.getPosition();
        Vector transformedPos = transformFieldOriented(otosPos);

        return new Pose2D(DistanceUnit.METER, initialPose.getX(DistanceUnit.METER) + transformedPos.x, initialPose.getY(DistanceUnit.METER) + transformedPos.y, AngleUnit.DEGREES, getFieldHeading(AngleUnit.DEGREES));
    }

}