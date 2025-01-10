
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.math_utils.Vector;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
//import org.firstinspires.ftc.teamcode.subsystems.components.Encoders;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.components.LED;
import org.firstinspires.ftc.teamcode.math_utils.PoseEstimator;

import java.util.Locale;


/**
 * Kiwi Teleop Code
 */
@TeleOp(name = "testTeleop", group = "Iterative OpMode")
public class TestTeleop extends OpMode {
    CompTeleop robot;
    double power;
    Controller controller1, controller2;
    DriveTrain drivetrain;
    Elevator elevator;
    Intake intake;
    //    Encoders encoder;
    LED leftLED, rightLED;
    boolean highGear = false;
    PoseEstimator poseEstimator;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {


        drivetrain = new DriveTrain(hardwareMap, new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0), 0, false);
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
//        drivetrain.init(hardwareMap);
        intake = new Intake(hardwareMap, telemetry);
        elevator = new Elevator(hardwareMap);
        poseEstimator = new PoseEstimator(hardwareMap, new Pose2D(DistanceUnit.INCH, -24, -63, AngleUnit.DEGREES, 90), true);




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
        if (controller1.dpadDown.isPressed()){

            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -57, -57, AngleUnit.DEGREES, 225),true);
        }
//
//
//
//
//        drivetrain.drive(new Vector(controller1.leftStick.x.value(), controller1.leftStick.y.value()), controller1.rightStick.x.value(), autoAlign, controller1.rightBumper.isPressed());
//        controller1.update();
//        controller2.update();
        drivetrain.update();
        telemetry.addData("elevator position", elevator.getElevatorPosition());
        telemetry.addData("intake arm position", intake.getIntakePosition());
        telemetry.addData("x", drivetrain.getPose().x);
        telemetry.addData("y", drivetrain.getPose().y);
        telemetry.addData("yaw", Math.toDegrees(drivetrain.getPose().angle));
        telemetry.addData("Robot Pose", drivetrain.getPose().toString());
        telemetry.addData("OTOS Heading", poseEstimator.otos.getHeading());
        telemetry.addData("OTOS Position", poseEstimator.otos.getPosition().toString());

        telemetry.addData("Initial Pose", String.format(Locale.US, "(%.3f, %.3f)", poseEstimator.initialPose.getX(DistanceUnit.INCH), poseEstimator.initialPose.getY(DistanceUnit.INCH)));
        telemetry.addData("Robot Pose", String.format(Locale.US, "(%.3f, %.3f)", poseEstimator.getPose().getX(DistanceUnit.INCH), poseEstimator.getPose().getY(DistanceUnit.INCH)));
        telemetry.addData("Using LL", poseEstimator.isUsingLL());






    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}