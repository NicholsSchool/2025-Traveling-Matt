package org.firstinspires.ftc.teamcode.subsystems.components;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuadBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;

public class OctoEncoder {

    private final OctoQuad oq;
    private final int encoderID;

    public OctoEncoder(HardwareMap hwMap, int encoderID, OctoQuadBase.EncoderDirection direction) {
        oq = hwMap.get(OctoQuad.class, "octoquad");
        this.encoderID = encoderID;
        oq.setSingleEncoderDirection(encoderID, direction);
    }

    public int getPosition() {
        return oq.readSinglePosition(encoderID);
    }

    public double getVelocity() {
        return (double) oq.readSingleVelocity(encoderID);
    }

}
