//package org.firstinspires.ftc.teamcode.subsystems.components;
//
//import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
//import com.qualcomm.hardware.digitalchickenlabs.OctoQuadBase;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.constants.OctoConstants;
//
//public class OctoEncoder implements OctoConstants {
//
//    private final OctoQuad oq;
//    private final int encoderID;
//
//    private int currentPosition;
//
//    private double currentVelocity;
//
//
//    public OctoEncoder(HardwareMap hwMap, int encoderID, OctoQuadBase.EncoderDirection direction) {
//        oq = hwMap.get(OctoQuad.class, "OctoQuad");
//        this.encoderID = encoderID;
//        oq.setSingleEncoderDirection(encoderID, direction);
//        oq.setChannelBankConfig(OctoQuadBase.ChannelBankConfig.ALL_QUADRATURE);
//    }
//
//    public int getPosition() {
//        return oq.readSinglePosition(encoderID);
//    }
//
//    public int getVelocity() {
//        return oq.readSingleVelocity(encoderID);
//    }
//
//    public void reset() {
//        oq.resetSinglePosition(encoderID);
//    }
//
//}
