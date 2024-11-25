package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
        outtakeRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        outtakeRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        outtakeLeft = hwMap.get(DcMotorEx.class, "outtakeLeft");
        outtakeLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        outtakeLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);




    }

    public int getOuttakePosition() {
        return outtakeLeft.getCurrentPosition();
    }

    public void outtakeSlideManual(double power){
        outtakeRight.setPower(-power);
        outtakeLeft.setPower(power);


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
