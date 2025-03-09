package org.firstinspires.ftc.teamcode.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
    LED leftLED, rightLED;
    boolean highGear = false;
    FtcDashboard dashboard;

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

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);


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

        controller1.update();
        controller2.update();
        drivetrain.update();
        elevator.periodic();
        //Run elevator - manual
        //ELEVATOR MANUAL
        if( Math.abs(controller2.leftStick.y.value()) > 0.05 ) {
            elevator.setState(Elevator.ELEVATOR_STATE.MANUAL);
            elevator.elevatorNoGovernor( controller2.leftStick.y.value() );
        } else { elevator.setState(Elevator.ELEVATOR_STATE.GO_TO_POSITION); }

        if(controller2.square.wasJustPressed()){
            elevator.elevatorToPos(ArmConstants.BUCKETHEIGHT);
        }

        if (controller2.square.wasJustReleased()){
            elevator.elevatorToPos(100);
        }

        //Run intake slide - manual
        intake.intakeSoftLimited(controller2.rightStick.y.value() * .8);

        //run intake, run outtake, or run nothing
        if (controller2.leftBumper.isPressed()){
            intake.intakeServo(.8);
        } else if (controller2.rightBumper.isPressed()){
            intake.outtakeBlock(.8);
        } else {
            telemetry.addData("nothing pressed", "true");
            intake.intakeServo(0);
            intake.outtakeBlock(0);
        }

        //reset sparkfun IMU
        if (controller1.options.isPressed()) {drivetrain.resetIMU();}
//
//        //go to basket height
//        if (controller2.square.isPressed()) {
//            elevator.elevatorToPos(ArmConstants.BUCKETHEIGHT);
//        }

        //intake all the way out
        if (controller2.circle.isPressed()) {
            intake.intakeToPos(31000);
        }

        //headlight at both basket heights, and half brightness for if it has a sample
        if (Math.abs(elevator.getElevatorPosition() - ArmConstants.BUCKETHEIGHT) < 1000) {
            elevator.headlight(1);

        } else if (Math.abs(elevator.getElevatorPosition() - 30000)< 1000){
            elevator.headlight(1);

        } else if (intake.hasSample()){
            elevator.headlight(0.5);
        } else{
            elevator.headlight(0);
        }

//        if(controller2.dpadRight.isPressed()) {
//            intake.wristGoToPos(200);
//        }

//        if(controller2.dpadLeft.isPressed()){
//            intake.setIntakeSetpoint(ArmConstants.INTAKE_DOWN);
//        }
//
        intake.periodic();

//        if (controller2.dpadRight.isPressed()) intake.setIntakeSetpoint(ArmConstants.INTAKE_DOWN);


        if (intake.getIntakeSlidePosition() < 12000) {
            intake.setWristSetpoint(ArmConstants.INTAKE_BUCKET);
        } else {
            intake.setWristSetpoint(intake.getIntakeSlidePosition() * ArmConstants.LINEAR_REGRESSION_M + ArmConstants.LINEAR_REGRESSION_B);
        }

        //intake slide encoder reset to 0
        if (controller2.dpadUp.isPressed()){
            intake.resetIntakeSlideposition();
        }

        if (controller2.dpadDown.isPressed()){
            intake.intakeSlideManual(-1);
        }

//        y=64.75233\cdot0.999976^{x}
        //intake.setIntakeSetpoint(ArmConstants.REGRESSION_M *  Math.pow(ArmConstants.REGRESSION_B, intake.getIntakeSlidePosition())); (exponential)

//        //manual reset elevator
//        if (controller2.dpadDown.isPressed()){
//            elevator.elevatorManual(-1);
//        }

//        //reset elevator encoder to 0
//      if (controller2.dpadUp.isPressed()){
//            elevator.resetElevatorposition();
//      }

      //auto align, 90, 0, 135, -90
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

        //drive
        drivetrain.drive(new Vector(controller1.leftStick.x.value(), controller1.leftStick.y.value()), controller1.rightStick.x.value(), autoAlign, controller1.rightBumper.isPressed());

        //update systems/controllers
        controller1.update();
        controller2.update();
        drivetrain.update();


        //Allllll the telemetry
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay()
                .setAlpha(0.7)
                .setStrokeWidth(1)
                .setStroke("Red")
                .strokeCircle(
                        drivetrain.getPose().getX(DistanceUnit.INCH),
                        drivetrain.getPose().getY(DistanceUnit.INCH),
                        9
                )
                .setStroke("Green")
                .strokeLine(
                        drivetrain.getPose().getX(DistanceUnit.INCH),
                        drivetrain.getPose().getY(DistanceUnit.INCH),
                        drivetrain.getPose().getX(DistanceUnit.INCH) + (9 * Math.cos(drivetrain.getPose().getHeading(AngleUnit.RADIANS))),
                        drivetrain.getPose().getY(DistanceUnit.INCH) + (9 * Math.sin(drivetrain.getPose().getHeading(AngleUnit.RADIANS)))
                );
        telemetry.addData("elevator position", elevator.getElevatorPosition());
        telemetry.addData("intake slide position", intake.getIntakeSlidePosition());
        telemetry.addData("x", drivetrain.getPose().getX(DistanceUnit.INCH));
        telemetry.addData("y", drivetrain.getPose().getY(DistanceUnit.INCH));
        telemetry.addData("yaw", Math.toDegrees(drivetrain.getPose().getHeading(AngleUnit.RADIANS)));
        telemetry.addData("Robot Pose", drivetrain.getPose().toString());
        telemetry.addData("headlights", elevator.getHeadlight());
        telemetry.addData("intake", intake.hasSample());
        telemetry.addData("intake power", intake.getIntakePower());
        telemetry.addData("intake position", intake.getWristPos());
        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}