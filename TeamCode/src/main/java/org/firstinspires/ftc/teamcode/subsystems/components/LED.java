package org.firstinspires.ftc.teamcode.subsystems.components;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.constants.LEDConstants;

public class LED implements LEDConstants {
    private final ServoImplEx IndLight;

    private final Colour defaultColour;

    private Thread sequenceThread;
    private Colour[] lastSequence;
    private int lastDelay;

    public LED(HardwareMap hwMap, String name, Colour defaultColour) {

        IndLight = hwMap.get(ServoImplEx.class, name);
        IndLight.setPwmRange(new PwmControl.PwmRange(500, 2500));
        IndLight.setPwmEnable();
        this.defaultColour = defaultColour;
        setColour(defaultColour);
    }

    public enum Colour {
        RED,
        ORANGE,
        YELLOW,
        GREEN,
        CYAN,
        BLUE,
        PURPLE,
        PINK,
        WHITE,
        BLACK
    }

    private double getPosForColour(@NonNull Colour colour) {
        switch (colour) {
            case RED:
                return IND_RED;
            case ORANGE:
                return IND_ORANGE;
            case YELLOW:
                return IND_YELLOW;
            case GREEN:
                return IND_GREEN;
            case CYAN:
                return IND_CYAN;
            case BLUE:
                return IND_BLUE;
            case PURPLE:
                return IND_PURPLE;
            case PINK:
                return IND_PINK;
            case WHITE:
                return IND_WHITE;
            case BLACK:
                return IND_BLACK;
            default:
                return IND_DEFAULT;
        }
    }

    public void setColour(Colour colour) {
        if (sequenceThread != null) { sequenceThread.interrupt(); }
        IndLight.setPosition(getPosForColour(colour));
    }
}
