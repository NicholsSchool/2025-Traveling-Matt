package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.math_utils.MotionProfile;
import org.firstinspires.ftc.teamcode.math_utils.RobotPose;
import org.firstinspires.ftc.teamcode.math_utils.SimpleFeedbackController;
import org.firstinspires.ftc.teamcode.math_utils.VectorMotionProfile;

public class Intake implements IntakeConstants {
    DcMotorEx slide;
    ServoImplEx wristOne, wristTwo;
    CRServoImplEx intakeOne,intakeTwo;
    ColorSensor colorSensor;
    boolean isBlueAlliance;

    public Intake(HardwareMap hwMap,  boolean isBlue) {


        slide = hwMap.get(DcMotorEx.class, "intakeSlide");
        slide.setDirection(DcMotorEx.Direction.FORWARD);
        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slide.setVelocityPIDFCoefficients(INTAKE_P, INTAKE_I, INTAKE_D, INTAKE_FF);


        isBlueAlliance = isBlue;

    }

    public void slide(double power){
        slide.setPower(power * SLIDE_SPEED);
    }

}
