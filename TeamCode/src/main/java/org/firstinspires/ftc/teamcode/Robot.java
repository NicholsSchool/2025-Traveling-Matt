package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;


/**
 * Kiwi Teleop Code
 */
@TeleOp(name="Robot", group="Iterative OpMode")
public class Robot extends OpMode
{
    Robot robot;
    double power;
    public Controller controller1, controller2;
    DriveTrain drivetrain;
    Outtake outtake;
    Intake intake;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        drivetrain = new DriveTrain();
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        drivetrain.init(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);


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
        drivetrain.fieldOriented(Math.hypot(controller1.leftStick.x.value(),controller1.leftStick.y.value()), -controller1.rightStick.x.value() * 0.3
        ,Math.toDegrees(Math.atan2(-controller1.leftStick.y.value(),controller1.leftStick.x.value())), 0);
        outtake.outtakeSlideManual(-controller2.leftStick.y.value());
        intake.intakeSlideManual(controller2.rightStick.y.value()*.7);
        //intake.intakeServo(controller2.leftBumper.isPressed() ? 1 : 0);
        intake.wristControl(controller2.leftBumper.isPressed());
        //intake.outtakeBlock(controller2.rightBumper.isPressed() ? 1 : 0 );
//        intake.intakeServo(controller2.leftTrigger.value() - controller2.rightTrigger.value());

        if(controller2.leftBumper.isPressed()) {
            intake.wristControl(controller2.leftBumper.isPressed());
            intake.intakeServo(1);


        }

        else if (controller2.rightBumper.isPressed()){
            intake.outtakeBlock(1);


        }

        else{
            telemetry.addData("nothing pressed", "true");
            intake.intakeServo(0);
            intake.outtakeBlock(0);
        }





 




        controller1.update();
        controller2.update();

        telemetry.addData("Outtake position", outtake.getOuttakePosition());








    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {}
}