package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.math_utils.Angles;
import org.firstinspires.ftc.teamcode.math_utils.VectorMotionProfile;
import org.firstinspires.ftc.teamcode.math_utils.MotionProfile;
import org.firstinspires.ftc.teamcode.math_utils.Vector;
import org.firstinspires.ftc.teamcode.math_utils.RobotPose;
import org.firstinspires.ftc.teamcode.math_utils.SimpleFeedbackController;
//import org.firstinspires.ftc.teamcode.subsystems.components.IndicatorLight;
import org.firstinspires.ftc.teamcode.subsystems.components.LED;
import org.firstinspires.ftc.teamcode.subsystems.components.OpticalSensor;
//import org.firstinspires.ftc.teamcode.subsystems.components.OctoEncoder;
import org.firstinspires.ftc.teamcode.math_utils.PoseEstimator;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuadBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;



/**
 * Robot Drivetrain
 */
public class DriveTrain implements DriveConstants {
    private final DcMotorEx leftDrive, rightDrive, backDrive;
//    private final OctoEncoder leftEncoder, rightEncoder, backEncoder;
    private final VectorMotionProfile driveProfile;
    private final MotionProfile turnProfile;
    private final SimpleFeedbackController turnController;
    private RobotPose pose;
    private final boolean isBlueAlliance;
    private double targetHeading, fieldOrientedForward;;
    private OpticalSensor od;
    public PoseEstimator poseEstimator;



    /**
     * Initializes the Drivetrain subsystem
     *
     * @param hwMap the hardwareMap
     * @param initialPose
     * @param isBlue whether we are blue alliance
     */
    public DriveTrain(HardwareMap hwMap, Pose2D initialPose, double fieldOrientedForward, boolean isBlue) {
        this.targetHeading = initialPose.getHeading(AngleUnit.RADIANS);
        this.fieldOrientedForward = Math.toRadians(fieldOrientedForward);
        this.isBlueAlliance = isBlue;
        od = new OpticalSensor("OTOS", hwMap, DistanceUnit.INCH, AngleUnit.RADIANS);
//        pose = new RobotPose(x, y, initialHeading);
        poseEstimator = new PoseEstimator(hwMap, initialPose, false);

        leftDrive = hwMap.get(DcMotorEx.class, "left");
        rightDrive = hwMap.get(DcMotorEx.class, "right");
        backDrive = hwMap.get(DcMotorEx.class, "front");

        leftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightDrive.setDirection(DcMotorEx.Direction.REVERSE);
        backDrive.setDirection(DcMotorEx.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        update();

             driveProfile = new VectorMotionProfile(DRIVE_PROFILE_SPEED);
        turnProfile = new MotionProfile(TURN_PROFILE_SPEED, TURN_PROFILE_MAX);
        turnController = new SimpleFeedbackController(AUTO_ALIGN_P);


    }

    public void update() {
        poseEstimator.update();

    }

    /**
     * Drives the robot field oriented
     *
     * @param driveInput the (x, y) input
     * @param turn the turning input
     * @param lowGear whether to put the robot to virtual low gear
     */
    public void drive(Vector driveInput, double turn, boolean autoAlign, boolean lowGear) {
        turn = 3.5 * (turnProfile.calculate(autoAlign ? turnToAngle() : turn));
        double turnCalculated = Math.abs(turn) < 0.05 ? turnProfile.calculate(turnToAngle()) : turn * 0.3;
        turnCalculated = turnCalculated * (lowGear ? 0.5 : 1.0);
        if(Math.abs(turn) < 0.05){
            setTargetHeading(getHeading(AngleUnit.RADIANS)
            );
        }
        driveInput = driveProfile.calculate(driveInput.clipMagnitude(
                (lowGear ? VIRTUAL_LOW_GEAR : VIRTUAL_HIGH_GEAR) - Math.abs(turn * 0.2)));
        double power = driveInput.magnitude();
        double angle = driveInput.angle();

        leftDrive.setPower(turnCalculated + power * Math.cos(angle + LEFT_DRIVE_OFFSET - poseEstimator.getPose().getHeading(AngleUnit.RADIANS)+ fieldOrientedForward));
        rightDrive.setPower(turnCalculated + power * Math.cos(angle + RIGHT_DRIVE_OFFSET - poseEstimator.getPose().getHeading(AngleUnit.RADIANS)+ fieldOrientedForward));
        backDrive.setPower(turnCalculated + power * Math.cos(angle + BACK_DRIVE_OFFSET - poseEstimator.getPose().getHeading(AngleUnit.RADIANS)+ fieldOrientedForward));
    }

    public boolean driveToPose(Pose2D targetPose, boolean lowGear){
        Vector driveInput = new Vector(targetPose.getX(DistanceUnit.INCH) - poseEstimator.getPose().getX(DistanceUnit.INCH),
                targetPose.getY(DistanceUnit.INCH) - poseEstimator.getPose().getY(DistanceUnit.INCH));
        if(driveInput.magnitude() < 1.5){
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            backDrive.setPower(0);
            return true;
        }
        setTargetHeading(targetPose.getHeading(AngleUnit.RADIANS));
        driveInput.scaleMagnitude(0.5);
        drive(driveInput,turnToAngle(), false, lowGear);
        return false;
    }

//    public void driveWithHeading(Vector driveInput, double turn, boolean lowGear, double desiredHeading) {
//        double turnCalculated = Math.abs(turn) < 0.05 ? turnProfile.calculate(turnToAngle()) : turn * 0.3;
//        if(Math.abs(turn) < 0.05){
//            setTargetHeading(getHeading(AngleUnit.RADIANS));
//        }
//        driveInput = driveProfile.calculate(driveInput.clipMagnitude(
//                (lowGear ? VIRTUAL_LOW_GEAR : VIRTUAL_HIGH_GEAR) - Math.abs(turn * 0.3)));
//        double power = driveInput.magnitude();
//        double angle = driveInput.angle();
//
//        while(Math.abs(desiredHeading - targetHeading) > 0.005) {
//            turn(-0.2);
//        }
//
//        leftDrive.setPower(turnCalculated + power * Math.cos(angle + LEFT_DRIVE_OFFSET - pose.angle));
//        rightDrive.setPower(turnCalculated + power * Math.cos(angle + RIGHT_DRIVE_OFFSET - pose.angle));
//        backDrive.setPower(turnCalculated + power * Math.cos(angle + BACK_DRIVE_OFFSET - pose.angle));
//    }

    private double turnToAngle() {
        double error = Angles.clipRadians(poseEstimator.getPose().getHeading(AngleUnit.RADIANS) - targetHeading);
        return Math.abs(error) < AUTO_ALIGN_ERROR ? 0.0 : turnController.calculate(error);
    }

    public void setTargetHeading(double targetHeading) {
        this.targetHeading = targetHeading;
    }

    public void setFloat() {
        leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        backDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

//    public void turnToAngle(double targetHeading){
//        double power = 2 * Math.sin(targetHeading - pose.angle);
//        if(Math.abs(targetHeading - pose.angle) > 0.001){
//            leftDrive.setPower(power);
//            rightDrive.setPower(power);
//            backDrive.setPower(power);
//        }
//
//    }


    public void turn(double power){
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        backDrive.setPower(power);
    }

    public void runDriveMotors(double power){
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        backDrive.setPower(power );

    }

    public Pose2D getPose() { return poseEstimator.getPose(); }

     public void resetIMU() {
        od.resetHeading();
        targetHeading = 0;
    }

    public double getHeading(AngleUnit unit){
        return poseEstimator.getPose().getHeading(unit);
    }



}