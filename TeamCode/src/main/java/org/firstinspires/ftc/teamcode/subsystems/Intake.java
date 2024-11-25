package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuadBase;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.components.OctoEncoder;

public class Intake implements IntakeConstants {
    public final DcMotorEx slide;
    public final OctoEncoder slideEncoder;
    public final ServoImplEx intakeWristF, intakeWristB;
    public final CRServoImplEx intakeOne,intakeTwo;
    //public final ColorSensor colorSensor;
    public final DigitalChannel slideMagnet;

    public Intake(HardwareMap hwMap) {
        slide = hwMap.get(DcMotorEx.class, "IntakeMotor");
        slide.setDirection(DcMotorEx.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        slideEncoder = new OctoEncoder(hwMap, SLIDE_ENC_ID, OctoQuadBase.EncoderDirection.FORWARD);

        intakeOne = hwMap.get(CRServoImplEx.class, "IntakeLeft");
        intakeTwo = hwMap.get(CRServoImplEx.class, "IntakeRight");

        intakeOne.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeWristF = hwMap.get(ServoImplEx.class, "WristFront");
        intakeWristB = hwMap.get(ServoImplEx.class, "WristBack");

        intakeWristF.setDirection(Servo.Direction.FORWARD);
        intakeWristB.setDirection(Servo.Direction.FORWARD);

        //colorSensor = hwMap.get(ColorSensor.class, "IntakeColor");

        slideMagnet = hwMap.get(DigitalChannel.class, "IntakeMagnet");

    }

    public void periodic() {
        slideEncoder.update();
        //if (slideMagnet.getState()) { slideEncoder.reset(); }
    }

    public int getEncoderTicks() { return slideEncoder.getPosition(); }

    public double[] getWristServoPositions() {
        return new double[]{intakeWristF.getPosition(), intakeWristB.getPosition()};
    }

    public void slideRawPower(double power){
        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide.setPower(power * SLIDE_SPEED);
    }

    public void slideToPos(double pos){
        //TODO: Use SimpleFeedbackController for this
    }

    public void runIntake(double power){
        intakeOne.setPower(power * INTAKE_SPEED);
        intakeTwo.setPower(power * INTAKE_SPEED);
    }

    public void wristToPos(double pos){
        intakeWristF.setPosition(pos);
        intakeWristB.setPosition(pos);
    }

}
