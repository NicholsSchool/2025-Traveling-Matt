package org.firstinspires.ftc.teamcode.subsystems;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;



import org.firstinspires.ftc.teamcode.constants.DriveConstants;

/**
 * This is NOT an op-mode.*
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.*
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 */
public class DriveTrain implements DriveConstants {
    /* Public OpMode members. */
    public DcMotor rearMotor, rightMotor, leftMotor;
    public RevBlinkinLedDriver blinkin;
    private AHRS navx;

    public boolean robotCentric = true;
    public boolean lowGear = false;

    /* local OpMode members. */
    HardwareMap hwMap;

    public void init( HardwareMap hwMap ) {

        // Saving a reference to the Hardware map...
        this.hwMap = hwMap;

        /*
         *  Motors
         */

        // Instantiating the Motors...
        rearMotor = hwMap.get( DcMotor.class, "Rear" );
        rightMotor = hwMap.get( DcMotor.class, "Right" );
        leftMotor = hwMap.get( DcMotor.class, "Left" );

        // Inverting the Motors...
        rearMotor.setDirection( DcMotorSimple.Direction.REVERSE );
        rightMotor.setDirection( DcMotorSimple.Direction.REVERSE );
        leftMotor.setDirection( DcMotorSimple.Direction.REVERSE );

        // Setting 0 Power Behavior...
        rearMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        rightMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        leftMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        navx = AHRS.getInstance(hwMap.get(NavxMicroNavigationSensor.class,
                "navX"), AHRS.DeviceDataType.kProcessedData);
        navx.zeroYaw();
    }


    /*
     *  MOTORS
     */

    /**
     * Moves the robot. No spinning, no saucing. Just linear-like, across-the-floor
     *  movement.
     *
     * @param speed the speed the robot will move at
     * @param angle the angle at which the robot will move
     */
    public void move( double speed, double angle ) {

        rearMotor.setPower( speed * Math.cos( Math.toRadians( angle + 180 ) ) );
        rightMotor.setPower( speed *  Math.cos( Math.toRadians( angle + 60 ) ) );
        leftMotor.setPower( speed * Math.cos( Math.toRadians( angle + 300 ) ) );
    }

//    public double getYaw(){
//        return angles.firstAngle;
//    }

    /**
     * Spins the robot. No linear movement, so no saucin'. Just spinning in place.
     *
     * @param speed the speed the robot will spin at
     */
    public void spin( double speed ) {

        if( robotCentric )
        {
            rearMotor.setPower( -speed );
            rightMotor.setPower( -speed );
            leftMotor.setPower( -speed );
        }
        else
        {
            rightMotor.setPower( -speed );
            leftMotor.setPower( -speed );
        }
    }

    /**
     * Combines spinning and linear-like movement to sauce. The robot will move across
     *  the floor while spinning. (Moves on a line, spinning all the while...)
     *
     * @param speed the speed the robot will move across the floor (on a line) at
     * @param spinSpeed the speed the robot will spin at
     * @param angle the angle at which the robot will move across the floor (on a line) at
     * @param deltaHeading how far, in degrees, the robot has spun on the elevated-floor-like
     *  axis. The change in Heading. Is updated with every loop
     */
    public void fieldOriented( double speed, double spinSpeed, double angle, double deltaHeading, boolean isHighGear)
    {
        double multiplier = (isHighGear ? 0.8 : 0.5);
        if( robotCentric )
        {
            rearMotor.setPower( (-spinSpeed + ( (speed * multiplier) * Math.cos( Math.toRadians( angle + 180 - deltaHeading ) ) ) ));
            rightMotor.setPower(  (-spinSpeed + ( (speed * multiplier) * Math.cos( Math.toRadians( angle + 60 - deltaHeading ) ) )) );
            leftMotor.setPower( -spinSpeed + ( (speed * multiplier) * Math.cos( Math.toRadians( angle + 300 - deltaHeading ) ) ) );
        }
        else
        {
            rearMotor.setPower( multiplier * ((speed * multiplier) * Math.cos( Math.toRadians( angle + 180 - deltaHeading) ) ) );
            rightMotor.setPower( multiplier * (-spinSpeed + ( (speed * multiplier) * Math.cos( Math.toRadians( angle + 60 - deltaHeading ) ) )) );
            leftMotor.setPower( multiplier * (-spinSpeed + ( (speed * multiplier) * Math.cos( Math.toRadians( angle + 300 - deltaHeading ) ) )) );
        }






    }


    /**
     * Stops the robot's 3 wheels. Do we need this??
     */
    public void stopWheels()
    {
        // Stopping Wheels...
        rearMotor.setPower( 0 );
        rightMotor.setPower( 0 );
        leftMotor.setPower( 0 );
    }

    public double getYaw(){
        return navx.getYaw();
    }

    public void resetYaw(){
        navx.zeroYaw();

    }

    /**
     * Resets the wheels' encoders. I think we need this :)
     */
    public void resetWheelEncoders()
    {
        rearMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        rightMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        leftMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

        rearMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
        rightMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
        leftMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
    }




}