package org.firstinspires.ftc.teamcode.subsystems.components;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuadBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.OctoConstants;

import java.util.HashMap;

public class OctoEncoder implements OctoConstants {

    private final OctoQuad oq;
    private final int encoderID;

    private int currentPosition;
    private int lastPosition;

    private double currentVelocity;
    private double lastVelocity;

    public OctoEncoder(HardwareMap hwMap, int encoderID, OctoQuadBase.EncoderDirection direction) {
        oq = hwMap.get(OctoQuad.class, "OctoQuad");
        this.encoderID = encoderID;
        oq.setSingleEncoderDirection(encoderID, direction);
        oq.setChannelBankConfig(OctoQuadBase.ChannelBankConfig.ALL_PULSE_WIDTH);
        oq.setSingleChannelPulseWidthParams(encoderID, new OctoQuad.ChannelPulseWidthParams(MIN_PULSE_LENGTH_US, MAX_PULSE_LENGTH_US));
    }

    public void update() {
        lastVelocity = currentVelocity;
        lastPosition = currentPosition;

        currentVelocity = oq.readSingleVelocity(encoderID);
        currentPosition = oq.readSinglePosition(encoderID);
    }

    public int getPosition() {
        return currentPosition;
    }

    public double getVelocity() {
        return currentVelocity;
    }

    public void reset() {
        oq.resetSinglePosition(encoderID);
    }

}
