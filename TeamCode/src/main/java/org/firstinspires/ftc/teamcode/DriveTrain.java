package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This is NOT an op-mode.*
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.*
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 */
public class DriveTrain implements DriveConstants{
    /* Public OpMode members. */

    public BNO055IMU imu;
    public DcMotor rearMotor, rightMotor, leftMotor;
    public RevBlinkinLedDriver blinkin;

    public boolean robotCentric = true;
    public boolean lowGear = false;

    /* local OpMode members. */
    HardwareMap hwMap;

    public void init( HardwareMap nowIknowmyabcshwMap ) {

        // Saving a reference to the Hardware map...
        this.hwMap = nowIknowmyabcshwMap;

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

        /*
         *  Blinkin
         */

        // Instantiating Blinkin...
        blinkin = hwMap.get( RevBlinkinLedDriver.class, "Blinkin" );


        /*
         *  IMU
         */

        // Instantiating IMU Parameters, setting angleUnit...
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        // Instantiating IMU... Initializing it with the above Parameters...
        imu = hwMap.get( BNO055IMU.class, "IMU" );
        imu.initialize( parameters );
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
    public void fieldOriented( double speed, double spinSpeed, double angle, double deltaHeading )
    {
        double sum = speed + spinSpeed;

        if( lowGear )
        {
            speed = ( speed / sum ) * 0.5;
            spinSpeed = ( spinSpeed / sum ) * 0.5;
        }
        else
        {
            speed /= sum;
            spinSpeed /= sum;
        }

        if( robotCentric )
        {
            rearMotor.setPower( -spinSpeed + ( speed * Math.cos( Math.toRadians( angle + 180 - deltaHeading ) ) ) );
            rightMotor.setPower( -spinSpeed + ( speed * Math.cos( Math.toRadians( angle + 60 - deltaHeading ) ) ) );
            leftMotor.setPower( -spinSpeed + ( speed * Math.cos( Math.toRadians( angle + 300 - deltaHeading ) ) ) );
        }
        else
        {
            rearMotor.setPower( speed * Math.cos( Math.toRadians( angle + 180 - deltaHeading ) ) );
            rightMotor.setPower( -spinSpeed + ( speed * Math.cos( Math.toRadians( angle + 60 - deltaHeading ) ) ) );
            leftMotor.setPower( -spinSpeed + ( speed * Math.cos( Math.toRadians( angle + 300 - deltaHeading ) ) ) );
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

    /**
     * Detects the colour we are currently on!
     * @return 1 if we're on blue, -1 if we're on red, else 0.
     */
    public Color colourSensorStuff(){
        // Pseudo-Code time !!
        //


        ColorSensor colorSensor;
        float[] hsvValues = new float[]{0F, 0F, 0F};

        // or: float hsvValues[] = {0F, 0F, 0F};
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        // bLedOn represents the state of the LED.
        boolean LedOn = true;

        // send the info back to driver station using telemetry function.
        // waitForStart();
        telemetry.addData("LED", LedOn ? "On" : "Off");
        telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Hue", hsvValues[0]);

        Color color;
        if(redThreshold < colorSensor.red())
            return Color.red;
        else if(blueThreshold < colorSensor.blue())
            return Color.blue;
        else
            return Color.neutral; // :)

    }
}