package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.ArmConstants;
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
@TeleOp(name = "Comp Teleop", group = "Iterative OpMode")
public class CompTeleop extends OpMode {
    CompTeleop robot;
    double power;
    public Controller controller1, controller2;
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

        drivetrain = new DriveTrain(hardwareMap, new Pose2D(DistanceUnit.INCH, -24, -63, AngleUnit.DEGREES, 90),  90, false);

        intake = new Intake(hardwareMap, telemetry);
        elevator = new Elevator(hardwareMap);

        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);


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

     */
    @Override
    public void start() {
//        while(!elevator.elevatorToPos(ArmConstants.AUTOELEVATORMIN)) {
//        }
//       elevator.resetElevatorposition();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if(!controller2.square.isPressed()) {
            elevator.elevatorSoftlimited(controller2.leftStick.y.value());
        }
        intake.intakeSoftLimited(controller2.rightStick.y.value() * .8);
        if (controller2.leftBumper.isPressed()){
            intake.intakeServo(1);
        } else if (controller2.rightBumper.isPressed()){
            intake.outtakeBlock(1);
        } else {
            telemetry.addData("nothing pressed", "true");
            intake.intakeServo(0);
            intake.outtakeBlock(0);
        }

        if (controller1.options.isPressed()) {drivetrain.resetIMU();}

        if (controller2.square.isPressed()) {
            elevator.elevatorToPos(ArmConstants.BUCKETHEIGHT);
        }

        if (controller2.circle.isPressed()) {
            intake.intakeToPos(-31000);
        }

        if (Math.abs(elevator.getElevatorPosition() - 52000)< 1000) {
            elevator.headlight(1);

        } else if (Math.abs(elevator.getElevatorPosition() - 30000)< 1000){
            elevator.headlight(1);

        } else{
            elevator.headlight(0);
        }

        if (controller2.dpadDown.isPressed()){
            elevator.elevatorManual(-1);
        }
      if (controller2.dpadUp.isPressed()){
            elevator.resetElevatorposition();
        }

        boolean autoAlign = controller1.rightStick.x.hasBeenZero();

        if(!autoAlign)
            drivetrain.setTargetHeading(drivetrain.getPose().getHeading(AngleUnit.RADIANS));
        else if(controller1.triangle.wasJustPressed())
            drivetrain.setTargetHeading(0);
        else if(controller1.x.wasJustPressed())
            drivetrain.setTargetHeading(3 *Math.PI / 4);
        else if(controller1.circle.wasJustPressed())
            drivetrain.setTargetHeading(-Math.PI / 2);
        else if(controller1.square.wasJustPressed())
            drivetrain.setTargetHeading(Math.PI / 2);

        drivetrain.drive(new Vector(controller1.leftStick.x.value(), controller1.leftStick.y.value()), controller1.rightStick.x.value(), autoAlign, controller1.rightBumper.isPressed());


        controller1.update();
        controller2.update();
        drivetrain.update();


        telemetry.addData("elevator position", elevator.getElevatorPosition());
        telemetry.addData("intake arm position", intake.getIntakePosition());
        telemetry.addData("x", drivetrain.getPose().getX(DistanceUnit.INCH));
        telemetry.addData("y", drivetrain.getPose().getY(DistanceUnit.INCH));
        telemetry.addData("yaw", Math.toDegrees(drivetrain.getPose().getHeading(AngleUnit.RADIANS)));
        telemetry.addData("Robot Pose", drivetrain.getPose().toString());
        telemetry.addData("elevatorTest", elevator.elevatorTest(57000));
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}