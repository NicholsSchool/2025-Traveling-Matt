package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.subsystems.components.Encoders;

public class Intake implements ArmConstants {
    DcMotorEx intakeSlide;
    Servo wristServo;
    CRServo intakeServo;
    Encoders encoder;

    public Intake(HardwareMap hwMap) {
        intakeServo = hwMap.get(CRServo.class, "intakeServo");
        intakeSlide = hwMap.get(DcMotorEx.class, "intakeSlide");
        wristServo = hwMap.get(Servo.class, "wristServo");
        encoder = new Encoders(hwMap);

        intakeSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }

    public void intakeSlideManual(double power){

        if (power < 0) {
            intakeSlide.setPower(power);
        }else{
            intakeSlide.setPower(0.7 * power);
        }



    }

    public void intakeSoftLimited(double power){
        if((power > 0 && encoder.getIntakePos() < INTAKEMAX) || (power <= 0 && encoder.getIntakePos() > INTAKEMIN)) {
            intakeSlideManual(power);
        }else{
            intakeSlide.setPower(0.0);

        }

        wristControl(encoder.getIntakePos() > INTAKEMAX - 10000);

    }



//    public void intakeSlidePos(int targetPos) {
//        intakeSlide.setPower((targetPos - intakeSlide.getCurrentPosition()) / 100.0);
//
//
//    }

    public void intakeServo(double power){
//        if(colorSensor.green() < 600) {
            intakeServo.setPower(power);
//        }else{
//            intakeServo.setPower(0);
        }


    public void wristControl(boolean isIntaking){
        wristServo.setPosition(isIntaking ? 0 : 1);

    }

    public void outtakeBlock(double power){
        intakeServo.setPower(-power);
    }

    public enum Color {
        RED,
        BLUE,
        YELLOW,
        NONE
    }
}
