package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Elevator.ELEVATOR_STATE.GO_TO_POSITION;
import static org.firstinspires.ftc.teamcode.subsystems.Elevator.ELEVATOR_STATE.MANUAL;

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
    PIDController elevatorPID;
    double elevatorSetpoint;
    Servo rightLight, leftLight;
    SimpleFeedbackController elevatorController;
    ELEVATOR_STATE state = MANUAL;

    public enum ELEVATOR_STATE {
        MANUAL,
        GO_TO_POSITION,
        STOPPED
    }

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
//        elevatorController = new SimpleFeedbackController(ELEVATOR_P);


        elevatorPID = new PIDController(ELEVATOR_P, 0.0, 0.0);

    }

    public void periodic() {

        switch (state) {
            case GO_TO_POSITION:
                elevatorNoGovernor(elevatorPID.calculate(getElevatorPosition(), elevatorSetpoint));
                break;
            case MANUAL:
                elevatorSetpoint = getElevatorPosition();
                break;
            case STOPPED:
            default:
                elevatorNoGovernor(0);
                break;
        }

    }


    public void elevatorNoGovernor(double power){
        elevatorLeft.setPower(power);
        elevatorRight.setPower(-power);
    }

    public void setState(ELEVATOR_STATE state)
    {
        this.state = state;
    }


    public boolean setSetpoint(double setpoint) {
//        this.state = GO_TO_POSITION;
        this.elevatorSetpoint = setpoint;
        if (Math.abs(setpoint - this.getElevatorPosition()) < 1000) {
            return true;
        } else { return false; }
    }

    public void elevatorSoftlimited(double power){

//        this.state = ELEVATOR_STATE.MANUAL;

        if (Math.abs(power) > 0.1) { state = MANUAL; }


        if((power > 0 && getElevatorPosition() < ELEVATORMAX) || (power <= 0 && getElevatorPosition() > ELEVATORMIN)) {
            elevatorRight.setPower(-power);
            elevatorLeft.setPower(power);
        }else{
            elevatorRight.setPower(0.0);
            elevatorLeft.setPower(0.0);
        }

    }

    public String getState(){
    switch (state) {
        case GO_TO_POSITION:
            return "gotopos";

        case MANUAL:
            return "manual";
        default:
            return "default";
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
//            elevatorNoGovernor(0);
            return true;
        }

        this.elevatorSetpoint = targetPos;
        state = GO_TO_POSITION;

        return false;
    }

//    public double elevatorTest(int targetPos){
//        return  (-Range.clip(elevatorController.calculate(getElevatorPosition() - targetPos), -1 , 1));
//    }

    public int getElevatorPosition(){
        return -elevatorLeft.getCurrentPosition();
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




    








