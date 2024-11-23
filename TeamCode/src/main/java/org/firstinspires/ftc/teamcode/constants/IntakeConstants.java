package org.firstinspires.ftc.teamcode.constants;

public interface IntakeConstants {
    double INTAKE_P = 0.1;
    double INTAKE_FEEDBACK_MULTIPLIER = 1;
    
    double SLIDE_SPEED = 0.3;
    int SLIDE_ENC_ID = 0;
    int SLIDE_TICKS = 100; //Measured in Ticks Per Centimeter

    double INTAKE_SPEED = 0.5;
    double INTAKE_DELTA_LENGTH = 1;
    double INTAKE_DELTA_THETA = 0.5;

    // Colour sensor constants for red sample
    int CSENS_RED_R = 1000;
    int CSENS_RED_G = 20;
    int CSENS_RED_B = 20;

    //Colour sensor constants for blue sample
    int CSENS_BLUE_R = 20;
    int CSENS_BLUE_G = 20;
    int CSENS_BLUE_B = 1000;
}
