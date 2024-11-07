package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.math_utils.MotionProfile;
import org.firstinspires.ftc.teamcode.math_utils.RobotPose;
import org.firstinspires.ftc.teamcode.math_utils.SimpleFeedbackController;
import org.firstinspires.ftc.teamcode.math_utils.VectorMotionProfile;

public class Intake implements IntakeConstants {
    DcMotorEx slide;
    ServoImplEx wristOne, wristTwo;
    CRServoImplEx intakeOne,intakeTwo;
    ColorSensor colorSensor;
    boolean isBlueAlliance;

    public Intake(HardwareMap hwMap,  boolean isBlue) {
        slide = hwMap.get(DcMotorEx.class, "intakeSlide");
        slide.setDirection(DcMotorEx.Direction.FORWARD);
        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intakeOne = hwMap.get(CRServoImplEx.class, "intakeOne");
        intakeTwo = hwMap.get(CRServoImplEx.class, "intakeTwo");

        intakeOne.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        wristOne = hwMap.get(ServoImplEx.class, "wristOne");
        wristTwo = hwMap.get(ServoImplEx.class, "wristTwo");

        wristOne.setDirection(Servo.Direction.FORWARD);
        wristTwo.setDirection(Servo.Direction.REVERSE);

        isBlueAlliance = isBlue;
    }

    public void slide(double power){
        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide.setPower(power * SLIDE_SPEED);
    }

    public void runIntake(double power){
        intakeOne.setPower(power * INTAKE_SPEED);
        intakeTwo.setPower(power * INTAKE_SPEED);
    }

    public void wristToPos(double pos){
        
    }

}
