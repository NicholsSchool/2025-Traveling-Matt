package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.math_utils.SimpleFeedbackController;
import org.firstinspires.ftc.teamcode.subsystems.components.Encoders;

public class Intake implements ArmConstants {
    DcMotorEx intakeSlide;
    Servo rightWrist, leftWrist;
    CRServo intakeServo;
    Telemetry telemetry;
    SimpleFeedbackController intakeController;
//

    public Intake(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intakeServo = hwMap.get(CRServo.class, "intakeServo");
        intakeSlide = hwMap.get(DcMotorEx.class, "intakeSlide");
        rightWrist = hwMap.get(Servo.class, "rightWrist");
        leftWrist = hwMap.get(Servo.class, "leftWrist");
//        encoder = new Encoders(hwMap);
        intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeController = new SimpleFeedbackController(INTAKE_P);

    }

    public void intakeSlideManual(double power){

        if (power < 0) {
            intakeSlide.setPower(power);
        }else{
            intakeSlide.setPower(power);
        }



    }

    public void intakeSoftLimited(double power){
        if((power > 0 && intakeSlide.getCurrentPosition() > INTAKEMAX) || (power <= 0 && intakeSlide.getCurrentPosition() < INTAKEMIN)) {
            intakeSlideManual(power);
        }else{
            intakeSlide.setPower(0.0);

        }

        telemetry.addData("isIntaking", intakeSlide.getCurrentPosition() < INTAKEMAX + 10000);
        telemetry.addData("Intaking limit", INTAKEMAX + 10000);
        wristControl(intakeSlide.getCurrentPosition() < INTAKEMAX + 22850);

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

    public boolean intakeToPos(int targetPos){
        if (Math.abs(getIntakePosition() - targetPos) < 1200){
            intakeSoftLimited(0);
            return true;
        }
        intakeSoftLimited(Range.clip(intakeController.calculate(getIntakePosition() - targetPos), -1    , 1));
        return false;
//        return true;
    }


    public void wristControl(boolean isIntaking){
        rightWrist.setPosition(isIntaking ? 1 : 0);
        leftWrist.setPosition(isIntaking ? 0 : 1);

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

    public int getIntakePosition(){
        return intakeSlide.getCurrentPosition();
    }
}
