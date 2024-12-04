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
import org.firstinspires.ftc.teamcode.subsystems.components.OctoEncoder;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuadBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;



/**
 * Robot Drivetrain
 */
public class DriveTrain implements DriveConstants {
    private final DcMotorEx leftDrive, rightDrive, backDrive;
    private final LED leftLED, rightLED;
//    private final OctoEncoder leftEncoder, rightEncoder, backEncoder;
    private final VectorMotionProfile driveProfile;
    private final MotionProfile turnProfile;
    private final SimpleFeedbackController turnController;
    private RobotPose pose;
    private RobotPose fieldPose;
    private RobotPose initalPose;
    private final boolean isBlueAlliance;
    private double imuOffset, targetHeading;
    private OpticalSensor od;

    /**
     * Initializes the Drivetrain subsystem
     *
     * @param hwMap the hardwareMap
     * @param x the initial x coordinate
     * @param y the initial y coordinate
     * @param initialHeading the initial robot heading in radians
     * @param isBlue whether we are blue alliance
     */
    public DriveTrain(HardwareMap hwMap, double x, double y, double initialHeading, boolean isBlue) {
        this.imuOffset = initialHeading + Math.PI;
        this.initalPose = new RobotPose(x, y, initialHeading + (isBlue ? Math.PI : 0));
        this.fieldPose = new RobotPose(x, y, initialHeading + (isBlue ? Math.PI : 0));
        this.targetHeading = initialHeading;
        this.isBlueAlliance = isBlue;
        od = new OpticalSensor("OTOS", hwMap, DistanceUnit.INCH, AngleUnit.RADIANS);
        pose = new RobotPose(x, y, initialHeading);


        leftDrive = hwMap.get(DcMotorEx.class, "Left");
        rightDrive = hwMap.get(DcMotorEx.class, "Right");
        backDrive = hwMap.get(DcMotorEx.class, "Rear");

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

        leftLED = new LED(hwMap, "leftLED", LED.Colour.BLUE);
        rightLED = new LED(hwMap, "rightLED", LED.Colour.BLUE);

//
//        leftDrive.setVelocityPIDFCoefficients(DRIVE_P, DRIVE_I, 0.0, 0.0);
//        rightDrive.setVelocityPIDFCoefficients(DRIVE_P, DRIVE_I, 0.0, 0.0);
//        backDrive.setVelocityPIDFCoefficients(DRIVE_P, DRIVE_I, 0.0, 0.0);
//
//
//        leftEncoder = new OctoEncoder(hwMap, LEFT_DRIVE_ENC, OctoQuadBase.EncoderDirection.FORWARD);
//        rightEncoder = new OctoEncoder(hwMap, RIGHT_DRIVE_ENC, OctoQuadBase.EncoderDirection.FORWARD);
//        backEncoder = new OctoEncoder(hwMap, BACK_DRIVE_ENC, OctoQuadBase.EncoderDirection.FORWARD);

//        leftLight = new IndicatorLight(hwMap, "LeftLight", IndicatorLight.Colour.GREEN);
//        rightLight = new IndicatorLight(hwMap, "RightLight", IndicatorLight.Colour.GREEN);
//
        driveProfile = new VectorMotionProfile(DRIVE_PROFILE_SPEED);
        turnProfile = new MotionProfile(TURN_PROFILE_SPEED, TURN_PROFILE_MAX);
        turnController = new SimpleFeedbackController(AUTO_ALIGN_P);
    }

    public void update() {
        od.update();
        pose = new RobotPose(od.getPosition().x, od.getPosition().y, od.getHeading() + imuOffset );
        fieldPose = new RobotPose(od.getPosition().x + initalPose.x, od.getPosition().y + initalPose.y, od.getHeading() + initalPose.angle );
    }

    /**
     * Drives the robot field oriented
     *
     * @param driveInput the (x, y) input
     * @param turn the turning input
     * @param lowGear whether to put the robot to virtual low gear
     */
    public void drive(Vector driveInput, double turn, boolean lowGear) {
        double turnCalculated = Math.abs(turn) < 0.05 ? turnProfile.calculate(turnToAngle()) : turn * 0.3;
        if(Math.abs(turn) < 0.05){
            setTargetHeading(pose.angle);
        }
        driveInput = driveProfile.calculate(driveInput.clipMagnitude(
                (lowGear ? VIRTUAL_LOW_GEAR : VIRTUAL_HIGH_GEAR) - Math.abs(turn * 0.3)));
        double power = driveInput.magnitude();
        double angle = driveInput.angle();

        leftDrive.setPower(turnCalculated + power * Math.cos(angle + LEFT_DRIVE_OFFSET - pose.angle));
        rightDrive.setPower(turnCalculated + power * Math.cos(angle + RIGHT_DRIVE_OFFSET - pose.angle));
        backDrive.setPower(turnCalculated + power * Math.cos(angle + BACK_DRIVE_OFFSET - pose.angle));
    }

    private double turnToAngle() {
        double error = Angles.clipRadians(pose.angle - targetHeading);
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



    public void runDriveMotors(double power){
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        backDrive.setPower(power );
    }

    public RobotPose getPose() { return fieldPose; }

    public void resetIMU() { od.resetHeading(); }
}