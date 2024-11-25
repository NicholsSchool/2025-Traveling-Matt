package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class liteTesting extends OpMode {

    private DcMotor leftClimbMotor, rightClimbMotor, intakeSlideMotor;
    private Servo wristServoF, wristServoB;
    private CRServo intakeServoL, intakeServoR, carriageServoL, carriageServoR;

    @Override
    public void init() {
        leftClimbMotor = hardwareMap.get(DcMotor.class, "LeftClimberMotor");
        rightClimbMotor = hardwareMap.get(DcMotor.class, "RightClimberMotor");
        intakeSlideMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");

        wristServoF = hardwareMap.get(Servo.class, "WristFront");
        wristServoB = hardwareMap.get(Servo.class, "WristBack");
        intakeServoL = hardwareMap.get(CRServo.class, "IntakeLeft");
        intakeServoR = hardwareMap.get(CRServo.class, "IntakeRight");
        carriageServoL = hardwareMap.get(CRServo.class, "LeftCarriage");
        carriageServoR = hardwareMap.get(CRServo.class, "RightCarriage");
    }

    @Override
    public void loop() {
        leftClimbMotor.setPower(gamepad1.left_stick_y);
        rightClimbMotor.setPower(gamepad1.left_stick_y);

        intakeSlideMotor.setPower(gamepad1.right_stick_y);

        wristServoF.setPosition(gamepad1.right_trigger);
        wristServoB.setPosition(gamepad1.right_trigger);

        intakeServoL.setPower(gamepad1.dpad_up ? 1 : 0);
        intakeServoR.setPower(gamepad1.dpad_up ? 1 : 0);

        carriageServoL.setPower(gamepad1.dpad_down ? 1 : 0);
        carriageServoR.setPower(gamepad1.dpad_down ? 1 : 0);
    }

}
