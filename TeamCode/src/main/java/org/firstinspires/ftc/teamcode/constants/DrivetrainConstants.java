package org.firstinspires.ftc.teamcode.constants;

/**
 * Constants for the Drivetrain
 */
public interface DrivetrainConstants {
    /** The Maximum Speed of the Driving Profile */
    double DRIVE_PROFILE_SPEED = 3.0;

    /** The Maximum Speed of the Turning Profile */
    double TURN_PROFILE_SPEED = 1.0;

    /** The Maximum Output Value Magnitude of the Turning Profile */
    double TURN_PROFILE_MAX = 0.25;

    /** Virtual Low Gear Max Speed */
    double VIRTUAL_LOW_GEAR = 0.375;

    /** Virtual High Gear Max Speed */
    double VIRTUAL_HIGH_GEAR = 0.75;

    /** Auto Align Proportional Constant */
    double AUTO_ALIGN_P = 0.45;

    /** Auto Align allowed error in radians (.5 degrees) */
    double AUTO_ALIGN_ERROR = 0.00872664625997;

    /** Left Drive Wheel Angle Offset (30 degrees) */
    double LEFT_DRIVE_OFFSET = Math.PI / 6.0;

    /** Right Drive Wheel Angle Offset (150 degrees) */
    double RIGHT_DRIVE_OFFSET = 5.0 * Math.PI / 6.0;

    /** Back Drive Wheel Angle Offset (270 degrees) */
    double BACK_DRIVE_OFFSET = 1.5 * Math.PI;

    /** Drive Motor Proportional Gain */
    double DRIVE_P = 16.0;

    /** Drive Motor Integral Gain */
    double DRIVE_I = 6.0;

    /** The Maximum Ticks per second velocity of 20:1 Hex Motors */
    double MAX_MOTOR_VELOCITY = 2800.0;

    /** Thru Bore Encoder ticks per revolution */
    int THRU_BORE_TICKS_PER_REV = 8192;

    /** Dead wheel diameter in inches */
    double DEAD_WHEEL_DIAMETER = 2.5;

    /** Inches per tick of a dead wheel */
    double INCHES_PER_TICK = DEAD_WHEEL_DIAMETER * Math.PI / THRU_BORE_TICKS_PER_REV;

    /** Horizontal Correction coefficient */
    double STRAFE_ODOMETRY_CORRECTION = 0.553336722097;

    /** Forward Correction coefficient */
    double FORWARD_ODOMETRY_CORRECTION = 0.942379475317;
}