package org.firstinspires.ftc.teamcode.subsystems.components;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.teamcode.constants.IndicatorConstants;

public class IndicatorLight implements IndicatorConstants {

    private final ServoImplEx IndLight;

    private final Colour defaultColour;
    
    private Thread sequenceThread;
    private Colour[] lastSequence;
    private int lastDelay;

    public IndicatorLight(HardwareMap hwMap, String name, Colour defaultColour) {

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
        sequenceThread.interrupt();
        IndLight.setPosition(getPosForColour(colour));
    }

//    public void setColourSequence(@NonNull Colour[] colours, int delay) {
//        if (colours == lastSequence && delay == lastDelay) return;
//
//        lastSequence = colours; lastDelay = delay;
//
//        if (sequenceThread != null) {
//            sequenceThread.interrupt();
//        }
//
//        sequenceThread = new Thread(() -> {
//            while (sequenceThread.isAlive()) {
//                for (Colour colour : colours) {
//                    setColour(colour);
//                    try { Thread.sleep(delay); } catch (InterruptedException e) {
//                        setColour(defaultColour);
//                        return;
//                    }
//                }
//            }
//        });
//        sequenceThread.start();
//    }
}
