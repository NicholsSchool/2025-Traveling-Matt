package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Outtake {
    DcMotorEx outtakeRight, outtakeLeft;


    /**
     *
     * @param hwMap
     */
    public Outtake(HardwareMap hwMap){
        //bucket slide is the vertical slide, intake is the horizontal slide
        outtakeRight = hwMap.get(DcMotorEx.class, "outtakeRight");
        outtakeLeft = hwMap.get(DcMotorEx.class, "outtakeLeft");




    }

    /**
     * This method takes a target position and moves the motors to that position
     *
     * @param targetPos is an integer where you set the position of the slides
     */
    public void outtakeToPos(int targetPos){
        outtakeRight.setPower((targetPos - outtakeRight.getCurrentPosition()) / 100.0);
        outtakeLeft.setPower((targetPos - outtakeLeft.getCurrentPosition()) / 100.0);

    }














}
