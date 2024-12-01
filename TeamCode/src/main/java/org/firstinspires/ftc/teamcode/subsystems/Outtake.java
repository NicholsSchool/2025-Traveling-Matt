package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.subsystems.components.Encoders;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuadBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.teamcode.math_utils.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.components.OctoEncoder;


public class Outtake implements ArmConstants {
    DcMotorEx outtakeRight, outtakeLeft;
    Encoders encoder;
    PIDController pidController;
    double setpoint;
    OctoEncoder slideEncoder;





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

        encoder = new Encoders(hwMap);
//        slideEncoder = new OctoEncoder(hwMap, SLIDE_ENC_ID, OctoQuadBase.EncoderDirection.FORWARD);
//        slideEncoder.reset();
//
//        setpoint = 0.0;
        pidController = new PIDController(0.0, 0.0, 0.0);




    }




    public void outtakeSlideManual(double power){
        if((power > 0 && encoder.getElevatorPos() < OUTTAKEMAX) || (power <= 0 && encoder.getElevatorPos() > OUTTAKEMIN)) {
            outtakeRight.setPower(power);
            outtakeLeft.setPower(-power);
        }else{
            outtakeRight.setPower(0.0);
            outtakeLeft.setPower(0.0);
        }

    }




    /**
     * This method takes a target position and moves the motors to that position
     *
     * @param targetPos is an integer where you set the position of the slides
     */
//    public void outtakeToPos(int targetPos){
//        outtakeRight.setPower((targetPos - outtakeRight.getCurrentPosition()) / 100.0);
//        outtakeLeft.setPower((targetPos - outtakeLeft.getCurrentPosition()) / 100.0);
//
//    }

    public void elevatorToPos(int targetPos){
        
    }




    







}
