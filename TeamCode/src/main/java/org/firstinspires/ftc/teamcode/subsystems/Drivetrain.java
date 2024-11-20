package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.math_utils.Angles;
import org.firstinspires.ftc.teamcode.math_utils.VectorMotionProfile;
import org.firstinspires.ftc.teamcode.math_utils.MotionProfile;
import org.firstinspires.ftc.teamcode.math_utils.Vector;
import org.firstinspires.ftc.teamcode.math_utils.RobotPose;
import org.firstinspires.ftc.teamcode.math_utils.SimpleFeedbackController;
import org.firstinspires.ftc.teamcode.subsystems.components.OpticalSensor;
import org.firstinspires.ftc.teamcode.subsystems.components.OctoEncoder;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuadBase;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;



/**
 * Robot Drivetrain
 */
public class Drivetrain implements DrivetrainConstants {
    private final DcMotorEx leftDrive, rightDrive, backDrive;
    private final OctoEncoder leftEncoder, rightEncoder, backEncoder;
    private final AHRS navx;
    private final VectorMotionProfile driveProfile;
    private final MotionProfile turnProfile;
    private final SimpleFeedbackController turnController;
    private final RobotPose pose;
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
    public Drivetrain(HardwareMap hwMap, double x, double y, double initialHeading, boolean isBlue) {
        this.imuOffset = initialHeading + (isBlue ? Math.PI : 0);
        this.targetHeading = initialHeading;
        od = new OpticalSensor("otos", hwMap, DistanceUnit.METER, AngleUnit.RADIANS);
        pose = new RobotPose(x, y, initialHeading);


        leftDrive = hwMap.get(DcMotorEx.class, "leftDrive");
        rightDrive = hwMap.get(DcMotorEx.class, "rightDrive");
        backDrive = hwMap.get(DcMotorEx.class, "backDrive");

        leftEncoder = new OctoEncoder(hwMap, LEFT_DRIVE_ENC, OctoQuadBase.EncoderDirection.FORWARD);
        rightEncoder = new OctoEncoder(hwMap, RIGHT_DRIVE_ENC, OctoQuadBase.EncoderDirection.FORWARD);
        backEncoder = new OctoEncoder(hwMap, BACK_DRIVE_ENC, OctoQuadBase.EncoderDirection.FORWARD);

        leftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightDrive.setDirection(DcMotorEx.Direction.REVERSE);
        backDrive.setDirection(DcMotorEx.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftDrive.setVelocityPIDFCoefficients(DRIVE_P, DRIVE_I, 0.0, 0.0);
        rightDrive.setVelocityPIDFCoefficients(DRIVE_P, DRIVE_I, 0.0, 0.0);
        backDrive.setVelocityPIDFCoefficients(DRIVE_P, DRIVE_I, 0.0, 0.0);

        navx = AHRS.getInstance(hwMap.get(NavxMicroNavigationSensor.class,
                "navx"), AHRS.DeviceDataType.kProcessedData);
        navx.zeroYaw();

        isBlueAlliance = isBlue;

        driveProfile = new VectorMotionProfile(DRIVE_PROFILE_SPEED);
        turnProfile = new MotionProfile(TURN_PROFILE_SPEED, TURN_PROFILE_MAX);
        turnController = new SimpleFeedbackController(AUTO_ALIGN_P);
    }

    /**
     * Drives the robot field oriented
     *
     * @param driveInput the (x, y) input
     * @param turn the turning input
     * @param autoAlign whether to autoAlign
     * @param lowGear whether to put the robot to virtual low gear
     */
    public void drive(Vector driveInput, double turn, boolean autoAlign, boolean lowGear) {
        turn = turnProfile.calculate(autoAlign ? turnToAngle() : turn);

        driveInput = driveProfile.calculate(driveInput.clipMagnitude(
                (lowGear ? VIRTUAL_LOW_GEAR : VIRTUAL_HIGH_GEAR) - Math.abs(turn)));
        double power = driveInput.magnitude();
        double angle = driveInput.angle();

        leftDrive.setPower(turn + power * Math.cos(angle + LEFT_DRIVE_OFFSET - pose.angle));
        rightDrive.setPower(turn + power * Math.cos(angle + RIGHT_DRIVE_OFFSET - pose.angle));
        backDrive.setPower(turn + power * Math.cos(angle + BACK_DRIVE_OFFSET - pose.angle));
    }

    private double turnToAngle() {
        double error = Angles.clipRadians(pose.angle - targetHeading);
        return Math.abs(error) < AUTO_ALIGN_ERROR ? 0.0 : turnController.calculate(error);
    }

    /**
     * Sets the auto-alignment target heading
     *
     * @param targetHeading the target heading in radians
     */
    public void setTargetHeading(double targetHeading) {
        this.targetHeading = targetHeading;
    }

    /**
     * Sets the Drive Wheels to Float Mode
     */
    public void setFloat() {
        leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        backDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Get Motor Velocities
     *
     * @return the left, right, back velocities
     */
    public double[] getMotorVelocities() {
        return new double[]{
                leftDrive.getVelocity(),
                rightDrive.getVelocity(),
                backDrive.getVelocity(),
        };
    }

    /**
     * Whether tne navx is connected and whether it is calibrating
     *
     * @return the information as a boolean array
     */
    public boolean[] getNavxInfo() {
        return new boolean[]{navx.isConnected(), navx.isCalibrating()};
    }

    /**
     * The pitch, roll, and yaw of the navx in degrees
     *
     * @return the information in degrees as a double array
     */
    public double[] getNavxAxes() {
        return new double[]{navx.getPitch(), navx.getRoll(), navx.getYaw()};
    }
}