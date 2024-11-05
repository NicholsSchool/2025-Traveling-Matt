package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Kiwi Teleop Code
 */
@TeleOp(name="Robot", group="Iterative OpMode")
public class Robot extends OpMode
{
    Robot robot;
    double power;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        DriveTrain driveTrain = new DriveTrain();
        driveTrain.init(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {}

    /*
     * Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop(){

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {}
}