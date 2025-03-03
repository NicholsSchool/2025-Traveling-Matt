package org.firstinspires.ftc.teamcode.constants;

public interface ArmConstants {
    //encoder IDs, I dont think we use these anymore
    int intakeSlideEncoderID = 4;
    int elevatorSlideEncoderID = 5;

    //the min and max limits on the elevator slides
    int ELEVATORMIN = 0;
    int ELEVATORMAX = 65000;

    //the min and max limits on the intake slides
    int INTAKEMIN = 0;
    int INTAKEMAX = 33000;

    //the elevator encoder heights for the top basket and ascent bar
    int BUCKETHEIGHT = 61800;
    int ASCENTHEIGHT = 17550;

    //these values are guesses, have to find correct ones
    double INTAKE_BUCKET = 300.0;
    double INTAKE_DOWN = 132.0;

    //PID coefficients
    double ELEVATOR_P = 0.001;
    double INTAKE_P = 0.0003;
    double INTAKEWRIST_P = 5E-3;
//5E-3
    double LINEAR_REGRESSION_M = 0.0035602;
    double LINEAR_REGRESSION_B = 48.14319;

    //-0.0027804
    //old -0.0036027
    //49.75741
    // old 48.14319

    double REGRESSION_M = 0.999976;
    double REGRESSION_B = 64.75233;


}
