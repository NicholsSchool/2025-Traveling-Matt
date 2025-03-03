package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.math_utils.SimpleFeedbackController;
import org.firstinspires.ftc.teamcode.subsystems.components.Encoders;
import org.firstinspires.ftc.teamcode.math_utils.PIDController;
//import org.firstinspires.ftc.teamcode.subsystems.components.OctoEncoder;


public class Elevator implements ArmConstants {
    DcMotorEx elevatorRight, elevatorLeft;
    Encoders encoder;
    PIDController pidController;
    double setpoint;
    Servo rightLight, leftLight;
    SimpleFeedbackController elevatorController;
//    OctoEncoder slideEncoder;






    /**
     *
     * @param hwMap
     */
    public Elevator(HardwareMap hwMap){
        //bucket slide is the vertical slide, intake is the horizontal slide
        elevatorRight = hwMap.get(DcMotorEx.class, "elevatorRight");
        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        elevatorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        elevatorLeft = hwMap.get(DcMotorEx.class, "elevatorLeft");
        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        elevatorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftLight = hwMap.get(Servo.class, "leftLight");
        rightLight = hwMap.get(Servo.class, "rightLight");
        elevatorController = new SimpleFeedbackController(ELEVATOR_P);


        pidController = new PIDController(0.0, 0.0, 0.0);




    }


    public void elevatorNoGovernor(double power){
        elevatorLeft.setPower(power);
        elevatorRight.setPower(-power);
    }

    public void elevatorManual(double power){
        elevatorNoGovernor(Range.clip(power, -ELEVATORMAX, ELEVATORMAX));

    }


    public void elevatorSoftlimited(double power){
        if((power > 0 && getElevatorPosition() < ELEVATORMAX) || (power <= 0 && getElevatorPosition() > ELEVATORMIN)) {
            elevatorRight.setPower(-power);
            elevatorLeft.setPower(power);
        }else{
            elevatorRight.setPower(0.0);
            elevatorLeft.setPower(0.0);
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

    public boolean elevatorToPos(int targetPos){
        if(Math.abs(getElevatorPosition() - targetPos) < 300){
            elevatorSoftlimited(0);
            return true;
        }
        elevatorSoftlimited(-Range.clip(elevatorController.calculate(getElevatorPosition() - targetPos), -1 , 1));
        return false;

    }

    public double elevatorTest(int targetPos){
        return  (-Range.clip(elevatorController.calculate(getElevatorPosition() - targetPos), -1 , 1));
    }

    public int getElevatorPosition(){
        return Math.abs(elevatorLeft.getCurrentPosition());
    }

    public void resetElevatorposition(){
        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void headlight(double brightness){
        leftLight.setPosition(brightness);
        rightLight.setPosition(brightness);

    }

    public double getHeadlight(){
        return leftLight.getPosition();
    }


}




    








