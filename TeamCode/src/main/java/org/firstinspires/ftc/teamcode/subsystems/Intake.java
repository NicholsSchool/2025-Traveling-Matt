package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.math_utils.PIDController;
import org.firstinspires.ftc.teamcode.math_utils.SimpleFeedbackController;

public class Intake implements ArmConstants {
    DcMotorEx intakeSlide;
    Telemetry telemetry;
    CRServoImplEx intakeRight, intakeLeft, wrist;
    SimpleFeedbackController intakeController;
    RevTouchSensor touchSensor;
    private final AnalogInput intakeEncoder;
    public final PIDController wristPID;
    private double intakeSetpoint;


    public Intake(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intakeRight = hwMap.get(CRServoImplEx.class, "intakeRight");
        intakeLeft = hwMap.get(CRServoImplEx.class, "intakeLeft");
        intakeSlide = hwMap.get(DcMotorEx.class, "intakeSlide");
        wrist = hwMap.get(CRServoImplEx.class, "rightWrist");
        intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeController = new SimpleFeedbackController(INTAKE_P);
        touchSensor = hwMap.get(RevTouchSensor.class, "touchSensor");
        intakeEncoder = hwMap.get(AnalogInput.class, "intakeEncoder");

        wristPID = new PIDController(INTAKEWRIST_P, 0.0, 0.0);

        intakeSetpoint = 0.0;

    }

    public void intakeSlideManual(double power){
        if (power < 0) {
            intakeSlide.setPower(power);
        }else{
            intakeSlide.setPower(power);
        }
    }

    //intake slide control, limited at 0 and the max extension
    public void intakeSoftLimited(double power){
        if((power > 0 && intakeSlide.getCurrentPosition() > INTAKEMAX) || (power <= 0 && intakeSlide.getCurrentPosition() < INTAKEMIN)) {
            intakeSlideManual(power);
        }else{
            intakeSlide.setPower(0.0);

        }

        telemetry.addData("isIntaking", intakeSlide.getCurrentPosition() < INTAKEMAX + 10000);
        telemetry.addData("Intaking limit", INTAKEMAX + 10000);
//        wristControl(intakeSlide.getCurrentPosition() < INTAKEMAX + 22850);

    }

    //run intake servo
    public void intakeServo(double power){
            intakeLeft.setPower(power);
            intakeRight.setPower( power);
    }

    //get the power value sent to the intake (for testing)
    public double getIntakePower(){
        return intakeLeft.getPower();
    }

    public double getIntakePosition() {
        return intakeEncoder.getVoltage() / 3.3 * 360.0;
    }

    public void setIntakeSetpoint(double setpoint) {
        this.intakeSetpoint = setpoint;
    }

//    public boolean wristGoToPos(double targetPos){
//        wrist.setPower(-0.5);
//        return (Math.abs(getIntakePosition() - targetPos) < 10);
//
//    }

    public void wristGoToPos(double targetPos){
        wristManual(wristPID.calculate(getIntakePosition(), targetPos));
    }

    public void wristManual(double power){
        wrist.setPower(power);
    }

    public void wristRegression (){

        if (getIntakeSlidePosition()> - 12000){
            getIntakePosition();

        }

    }


    //mainly for autos, run intakeSlide out to a position
    public boolean intakeToPos(int targetPos){
        if (Math.abs(getIntakePosition() - targetPos) < 1200){
            intakeSoftLimited(0);
            return true;
        }
        intakeSoftLimited(Range.clip(intakeController.calculate(getIntakePosition() - targetPos), -1    , 1));
        return false;
    }

    //returns true if the touch sensor is pressed
    public boolean hasSample(){
       return touchSensor.isPressed();
    }


//    public void wristControl(boolean isIntaking){
//        wrist.setPosition(isIntaking ? 1 : 0);
//
//    }




    //outtakes a block
    public void outtakeBlock(double power){
        intakeRight.setPower(-power);
        intakeLeft.setPower(-power);
    }

    //we don't need this I think
    public enum Color {
        RED,
        BLUE,
        YELLOW,
        NONE
    }

    //get the current position of the intake slides
    public int getIntakeSlidePosition(){
        return intakeSlide.getCurrentPosition();
    }
}
