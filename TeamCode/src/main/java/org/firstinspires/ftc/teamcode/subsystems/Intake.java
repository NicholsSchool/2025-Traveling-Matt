package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    DcMotorEx intakeSlide;
    Servo wristServo;
    CRServo intakeServo;
    ColorSensor colorSensor;

    public Intake(HardwareMap hwMap) {
        intakeServo = hwMap.get(CRServo.class, "intakeServo");
        intakeSlide = hwMap.get(DcMotorEx.class, "intakeSlide");
        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");
        wristServo = hwMap.get(Servo.class, "wristServo");

    }

    public void intakeSlideManual(double power){
        intakeSlide.setPower(power);


    }

    public void intakeSlidePos(int targetPos) {
        intakeSlide.setPower((targetPos - intakeSlide.getCurrentPosition()) / 100.0);


    }

    public void intakeServo(double power){
        intakeServo.setPower(power);
  }

    public void wristControl(boolean isIntaking){
        wristServo.setPosition(isIntaking ? 0 : 1);

    }

    public void outtakeBlock(double power){
        intakeServo.setPower(-power);
    }

    public int color() {
        int red = colorSensor.red();
        int blue = colorSensor.blue();
        int green = colorSensor.green();
        //yellow is a combination of rgb and i don't know how to do it

        if (red > 450) {
            return 2;
        }else if (green > 550){
            return 3;
        }else if (blue > 1){
            return 4;
        }else {
            return 1;

        }
    }



}
