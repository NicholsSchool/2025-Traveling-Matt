package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    DcMotorEx intakeSlide;
    Servo leftIntakeWheel, rightIntakeWheel;
    ColorSensor colorSensor;

    public Intake(HardwareMap hwMap) {
        leftIntakeWheel = hwMap.get(Servo.class, "leftIWheel");
        rightIntakeWheel = hwMap.get(Servo.class, "rightIWheel");
        intakeSlide = hwMap.get(DcMotorEx.class, "intakeWheel");
        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");

    }

    public void intakeSlidePos(int targetPos) {
        intakeSlide.setPower((targetPos - intakeSlide.getCurrentPosition()) / 100.0);


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
