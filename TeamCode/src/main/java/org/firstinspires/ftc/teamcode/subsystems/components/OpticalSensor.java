package org.firstinspires.ftc.teamcode.subsystems.components;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.math_utils.Vector;

public class OpticalSensor {

    private final SparkFunOTOS otos;
    private SparkFunOTOS.Pose2D pos;

    /**
     * Instantiates the Sparkfun OTOS Sensor
     * @param deviceName Name set in HardwareMap Config
     * @param hwMap HardwareMap passthrough from TeleOp
     * @param linearUnit Unit to measure linear motion (DistanceUnit.METERS or DistanceUnit.INCHES)
     * @param angularUnit Unit to measure angle (AngleUnit.DEGREES or AngleUnit.RADIANS)
     */
    public OpticalSensor(String deviceName, @NonNull HardwareMap hwMap, DistanceUnit linearUnit, AngleUnit angularUnit) {

        otos = hwMap.get(SparkFunOTOS.class, deviceName);

        otos.begin();
        otos.resetTracking();
        otos.setLinearUnit(linearUnit);
        otos.setAngularUnit(angularUnit);
        otos.calibrateImu();

    }

    /**
     * Updates the sensor position data, run every loop when using getPosition or getHeading.
     */
    public void update() {
        pos = otos.getPosition();
    }

    /**
     * Returns the position of the robot as a Vector
     * @return Robot Position Vector
     */
    public Vector getPosition() {
        return new Vector(pos.x, pos.y);
    }

    /**
     * Returns the heading of the robot
     * @return Robot Heading
     */
    public double getHeading() {
        return pos.h;
    }

    public void resetHeading() { otos.setPosition( new SparkFunOTOS.Pose2D( pos.x, pos.y, 0.0 ) ); }

}