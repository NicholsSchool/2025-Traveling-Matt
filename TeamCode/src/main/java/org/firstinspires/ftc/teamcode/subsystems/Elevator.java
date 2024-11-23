package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuadBase;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.ElevatorConstants;
import org.firstinspires.ftc.teamcode.subsystems.components.OctoEncoder;


/**
 * Robot Arm
 */
public class Elevator implements ElevatorConstants {
    private final DcMotorEx leftSlideMotor, rightSlideMotor;
    private final OctoEncoder slideEncoder;
    private final CRServoImplEx leftCarriageServo;
    private final CRServoImplEx rightCarriageServo;
    private final ColorSensor carriageSensor;
    private final DigitalChannel slideMagnet;

    /**
     * Initializes the Arm
     *
     * @param hardwareMap the hardware map
     */
    public Elevator(HardwareMap hardwareMap) {
        leftSlideMotor = hardwareMap.get(DcMotorEx.class, "leftClimberMotor");
        leftSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftSlideMotor.setDirection(DcMotorEx.Direction.FORWARD);

        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightClimberMotor");
        rightSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setDirection(DcMotorEx.Direction.FORWARD);

        leftCarriageServo = hardwareMap.get(CRServoImplEx.class, "leftCarriage");
        leftCarriageServo.setDirection(CRServoImplEx.Direction.FORWARD);

        rightCarriageServo = hardwareMap.get(CRServoImplEx.class, "rightCarriage");
        rightCarriageServo.setDirection(CRServoImplEx.Direction.REVERSE);

        slideEncoder = new OctoEncoder(hardwareMap, SLIDE_ENC_ID, OctoQuadBase.EncoderDirection.FORWARD);

        carriageSensor = hardwareMap.get(ColorSensor.class, "CarriageColor");

        slideMagnet = hardwareMap.get(DigitalChannel.class, "LeftClimberMagnet");
    }

    public void periodic() {
        if (slideMagnet.getState()) { slideEncoder.reset(); }
    }

    public int getEncoderTicks() { return slideEncoder.getPosition(); }

    public void setSlidePosition(double inputPosition) {
        //TODO: Use SimpleFeedbackController for this
    }

    public void slideRawPower(double power){
        leftSlideMotor.setPower(power);
        rightSlideMotor.setPower(power);
    }

    public void setCarriageServoPower(double inputPower) {
        leftCarriageServo.setPower(inputPower);
        rightCarriageServo.setPower(inputPower);
    }
}