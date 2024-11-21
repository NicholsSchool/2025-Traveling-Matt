package org.firstinspires.ftc.teamcode.constants;

import org.firstinspires.ftc.teamcode.subsystems.components.IndicatorLight;

public interface IndicatorConstants {

    double IND_DEFAULT = 0.0;

    double IND_BLACK = 0.25;

    double IND_RED = 0.30;

    double IND_ORANGE = 0.33;

    double IND_YELLOW = 0.38;

    double IND_GREEN = 0.49;

    double IND_CYAN = 0.55;

    double IND_BLUE = 0.60;

    double IND_PURPLE = 0.66;

    double IND_PINK = 0.69;

    double IND_WHITE = 0.75;

    IndicatorLight.Colour[] GREEN_FLASH = new IndicatorLight.Colour[]{IndicatorLight.Colour.GREEN,IndicatorLight.Colour.BLACK};

    IndicatorLight.Colour[] GREEN_YELLOW = new IndicatorLight.Colour[]{IndicatorLight.Colour.GREEN, IndicatorLight.Colour.YELLOW};

    IndicatorLight.Colour[] RED_FLASH = new IndicatorLight.Colour[]{IndicatorLight.Colour.RED, IndicatorLight.Colour.BLACK};

}
