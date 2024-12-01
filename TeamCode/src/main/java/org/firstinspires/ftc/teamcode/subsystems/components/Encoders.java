package org.firstinspires.ftc.teamcode.subsystems.components;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuadBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.ArmConstants;

public class Encoders implements ArmConstants {
    private OctoQuad octoquad;

    private int         elevatorPos;
    private int         intakePos;


    public Encoders(HardwareMap hwMap){
        octoquad = hwMap.get(OctoQuad.class, "octoquad");

        octoquad.setSingleEncoderDirection(intakeSlideEncoderID, OctoQuadBase.EncoderDirection.FORWARD);
        octoquad.setSingleEncoderDirection(elevatorSlideEncoderID, OctoQuadBase.EncoderDirection.FORWARD);

        octoquad.saveParametersToFlash();

        octoquad.resetAllPositions();

    }

    public int getIntakePos(){
      return octoquad.readSinglePosition(intakeSlideEncoderID);

    }

    public int getElevatorPos(){
        return octoquad.readSinglePosition(elevatorSlideEncoderID);
    }
}
