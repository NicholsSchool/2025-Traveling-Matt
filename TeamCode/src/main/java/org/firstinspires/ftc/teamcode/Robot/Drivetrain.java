package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.Constants;

public class Drivetrain implements Constants {
    private final DcMotorEx rightDrive, leftDrive, backDrive;
    Telemetry telemetry;

    /**
     * Initializes the Drivetrain
     *
     * @param hwMap the HardwareMap of the robot
     * @param telemetry the Telemetry to be used later
     */
    public Drivetrain(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        rightDrive = hwMap.get(DcMotorEx.class, "rightDrive");
        leftDrive = hwMap.get(DcMotorEx.class, "leftDrive");
        backDrive = hwMap.get(DcMotorEx.class, "backDrive");

        rightDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        backDrive.setDirection(DcMotorEx.Direction.REVERSE);

        rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Drives the robot
     *
     * @param power the power input
     * @param turn the turn input
     * @param angle the angle input
     */
    public void drive(double power, double turn, double angle) {
        rightDrive.setPower(turn + power * Math.cos(angle) + RIGHT_DRIVE_OFFSET);
        leftDrive.setPower(turn + power * Math.cos(angle) + LEFT_DRIVE_OFFSET);
        backDrive.setPower(turn + power * Math.cos(angle) + BACK_DRIVE_OFFSET);
    }
}
