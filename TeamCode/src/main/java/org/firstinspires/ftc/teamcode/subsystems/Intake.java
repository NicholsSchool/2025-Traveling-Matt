package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.constants.IntakeConstants;

public class Intake implements IntakeConstants {
    DcMotorEx slide;
    ServoImplEx intakeWristF, intakeWristB;
    CRServoImplEx intakeOne,intakeTwo;
    ColorSensor colorSensor;
    DigitalChannel intakeMagnet;
    boolean isBlueAlliance;

    public Intake(HardwareMap hwMap,  boolean isBlueAlliance) {
        slide = hwMap.get(DcMotorEx.class, "IntakeSlideMotor");
        slide.setDirection(DcMotorEx.Direction.FORWARD);
        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intakeOne = hwMap.get(CRServoImplEx.class, "IntakeLeftWheel");
        intakeTwo = hwMap.get(CRServoImplEx.class, "IntakeRightWheel");

        intakeOne.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeWristF = hwMap.get(ServoImplEx.class, "IntakeWristFront");
        intakeWristB = hwMap.get(ServoImplEx.class, "IntakeWristBack");

        intakeWristF.setDirection(Servo.Direction.FORWARD);
        intakeWristB.setDirection(Servo.Direction.REVERSE);

        colorSensor = hwMap.get(ColorSensor.class, "IntakeColor");

        intakeMagnet = hwMap.get(DigitalChannel.class, "IntakeMagnet");

        this.isBlueAlliance = isBlueAlliance;
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
