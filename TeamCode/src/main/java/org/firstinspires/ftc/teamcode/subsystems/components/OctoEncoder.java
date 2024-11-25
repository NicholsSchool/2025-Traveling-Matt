package org.firstinspires.ftc.teamcode.subsystems.components;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuadBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.OctoConstants;
import org.firstinspires.ftc.teamcode.math_utils.EncoderFallingEdge;

import java.util.HashMap;

public class OctoEncoder implements OctoConstants {

    private final OctoQuad oq;
    private final int encoderID;

    private int currentPosition;

    private double currentVelocity;

    private EncoderFallingEdge encoderFallingEdge;


    public OctoEncoder(HardwareMap hwMap, int encoderID, OctoQuadBase.EncoderDirection direction) {
        oq = hwMap.get(OctoQuad.class, "OctoQuad");
        this.encoderID = encoderID;
        oq.setSingleEncoderDirection(encoderID, direction);
        oq.setChannelBankConfig(OctoQuadBase.ChannelBankConfig.ALL_PULSE_WIDTH);
        oq.setSingleChannelPulseWidthParams(encoderID, new OctoQuad.ChannelPulseWidthParams(MIN_PULSE_LENGTH_US, MAX_PULSE_LENGTH_US));
        encoderFallingEdge = new EncoderFallingEdge( this::getRawPosition, this::getRawVelocity );
    }

    public int getPosition() {
        return encoderFallingEdge.calculatePosition();
    }

    public void reset() {
        oq.resetSinglePosition(encoderID);
        encoderFallingEdge.reset();
    }

    private int getRawPosition() {
        return oq.readSinglePosition(encoderID);
    }

    private int getRawVelocity() {
        return Short.valueOf(oq.readSingleVelocity(encoderID)).intValue();
    }

}
