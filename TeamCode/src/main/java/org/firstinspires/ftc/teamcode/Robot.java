package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.math_utils.Vector;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
//import org.firstinspires.ftc.teamcode.subsystems.components.Encoders;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.components.LED;


/**
 * Kiwi Teleop Code
 */
@TeleOp(name = "Robot", group = "Iterative OpMode")
public class Robot extends OpMode {
    Robot robot;
    double power;
    public Controller controller1, controller2;
    DriveTrain drivetrain;
    Outtake outtake;
    Intake intake;
//    Encoders encoder;
    LED leftLED, rightLED;
    boolean highGear = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        drivetrain = new DriveTrain(hardwareMap, 0, 0, Math.PI / 2, false);
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
//        drivetrain.init(hardwareMap);
        intake = new Intake(hardwareMap, telemetry);
        outtake = new Outtake(hardwareMap);
//        encoder = new Encoders(hardwareMap);



        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
        drivetrain.drive(new Vector(controller1.leftStick.x.value(), controller1.leftStick.y.value()), controller1.rightStick.x.value(), controller1.rightBumper.isPressed());
        outtake.outtakeSlideManual(controller2.leftStick.y.value());
        intake.intakeSoftLimited(controller2.rightStick.y.value());
        if (controller2.leftBumper.isPressed()) {
//            intake.wristControl(controller2.leftBumper.isPressed());
            intake.intakeServo(1);
        } else if (controller2.rightBumper.isPressed()) {
            intake.outtakeBlock(1);


        } else {
            telemetry.addData("nothing pressed", "true");
            intake.intakeServo(0);
            intake.outtakeBlock(0);
        }

        if (controller1.options.isPressed()) {
            drivetrain.resetIMU();
        }

        if (controller2.square.isPressed()) {
            outtake.elevatorToPos(50000);
        }


        controller1.update();
        controller2.update();
        drivetrain.update();


        telemetry.addData("elevator position", intake.getIntakePosition());
        telemetry.addData("intake arm position", outtake.getOuttakePosition());
        telemetry.addData("x", drivetrain.getPose().x);
        telemetry.addData("y", drivetrain.getPose().y);
        telemetry.addData("yaw", drivetrain.getPose().angle * 180 / Math.PI);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}