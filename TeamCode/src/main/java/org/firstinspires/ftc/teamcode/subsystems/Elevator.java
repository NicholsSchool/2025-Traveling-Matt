package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuadBase;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.ElevatorConstants;
import org.firstinspires.ftc.teamcode.subsystems.components.OctoEncoder;


/**
 * Robot Arm
 */
public class Elevator implements ElevatorConstants {
    private final DcMotorEx leftSlideMotor, rightSlideMotor;
    private final OctoEncoder slideEncoder;
    private final CRServoImplEx leftWheelServo;
    private final CRServoImplEx rightWheelServo;

    /**
     * Initializes the Arm
     *
     * @param hardwareMap the hardware map
     */
    public Elevator(HardwareMap hardwareMap) throws Exception {
        leftSlideMotor = hardwareMap.get(DcMotorEx.class, "leftShoulder");
        leftSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftSlideMotor.setDirection(DcMotorEx.Direction.FORWARD);

        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightShoulder");
        rightSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setDirection(DcMotorEx.Direction.FORWARD);

        leftWheelServo = hardwareMap.get(CRServoImplEx.class, "leftWheelServo");
        leftWheelServo.setDirection(CRServoImplEx.Direction.FORWARD);

        rightWheelServo = hardwareMap.get(CRServoImplEx.class, "rightWheelServo");
        rightWheelServo.setDirection(CRServoImplEx.Direction.REVERSE);

        slideEncoder = new OctoEncoder(hardwareMap, SLIDE_ENC_ID, OctoQuadBase.EncoderDirection.FORWARD);

    }


    public void setSlideVelocity(double inputVelocity) {

    }

    public void setSlidePosition(double inputPosition) {

    }

    public void setCarriageServoSpeed(double inputSpeed) {
        leftWheelServo.setPower(inputSpeed);
        rightWheelServo.setPower(inputSpeed);
    }
}