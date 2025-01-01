
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.math_utils.Vector;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
//import org.firstinspires.ftc.teamcode.subsystems.components.Encoders;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.components.LED;


/**
 * Kiwi Teleop Code
 */
@TeleOp(name = "testTeleop", group = "Iterative OpMode")
public class TestTeleop extends OpMode {
    Robot robot;
    double power;
    Controller controller1, controller2;
    DriveTrain drivetrain;
    Elevator elevator;
    Intake intake;
    //    Encoders encoder;
    LED leftLED, rightLED;
    boolean highGear = false;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {


        drivetrain = new DriveTrain(hardwareMap, 0, 0, 0, false);
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
//        drivetrain.init(hardwareMap);
        intake = new Intake(hardwareMap, telemetry);
        elevator = new Elevator(hardwareMap);




    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (controller1.square.isPressed()){
            drivetrain.testLeft(1);
        }

        drivetrain.badDrive();
        elevator.headlight(1);


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}