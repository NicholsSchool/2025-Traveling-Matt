
package org.firstinspires.ftc.teamcode.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.controller.Controller;
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
    FtcDashboard dashboard;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        Pose2D initialPose = new Pose2D(DistanceUnit.INCH, 0, -48, AngleUnit.DEGREES, 90);
        drivetrain = new DriveTrain(hardwareMap, initialPose, 90, false );
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
//        drivetrain.init(hardwareMap);
        intake = new Intake(hardwareMap, telemetry);
        elevator = new Elevator(hardwareMap);
        poseEstimator = new PoseEstimator(hardwareMap, new Pose2D(DistanceUnit.INCH, 0, -48, AngleUnit.DEGREES, 90), false);
        dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);




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


        if(controller2.square.isPressed()){
            elevator.elevatorToPos(17000);
        }


        if(controller2.square.isPressed()){
            elevator.elevatorToPos(ArmConstants.BUCKETHEIGHT);
        }

        elevator.periodic();


        telemetry.addData("OTOS Heading", poseEstimator.otos.getHeading());
        telemetry.addData("OTOS Position", poseEstimator.otos.getPosition().toString());

        telemetry.addData("Initial Pose", String.format(Locale.US, "(%.3f, %.3f)", poseEstimator.initialPose.getX(DistanceUnit.INCH), poseEstimator.initialPose.getY(DistanceUnit.INCH)));
        telemetry.addData("Robot Pose", String.format(Locale.US, "(%.3f, %.3f)", poseEstimator.getPose().getX(DistanceUnit.INCH), poseEstimator.getPose().getY(DistanceUnit.INCH)));
        telemetry.addData("Using LL", poseEstimator.isUsingLL());

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay()
                .setAlpha(0.7)
                .setStrokeWidth(1)
                .setStroke("Red")
                .strokeCircle(
                        poseEstimator.getPose().getX(DistanceUnit.INCH),
                        poseEstimator.getPose().getY(DistanceUnit.INCH),
                        9
                )
                .setStroke("Green")
                .strokeLine(
                        poseEstimator.getPose().getX(DistanceUnit.INCH),
                        poseEstimator.getPose().getY(DistanceUnit.INCH),
                        poseEstimator.getPose().getX(DistanceUnit.INCH) + (9 * Math.cos(poseEstimator.getPose().getHeading(AngleUnit.RADIANS))),
                        poseEstimator.getPose().getY(DistanceUnit.INCH) + (9 * Math.sin(poseEstimator.getPose().getHeading(AngleUnit.RADIANS)))
                );
        dashboard.sendTelemetryPacket(packet);
        telemetry.update();






//        if (controller2.dpadDown.isPressed()){
//            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 20, -48, AngleUnit.DEGREES, 0),false, poseEstimator.getPose());
//
////            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -57, -57, AngleUnit.DEGREES, 0),true, poseEstimator.getPose() );
//        }
//
//
//
//
//        drivetrain.drive(new Vector(controller1.leftStick.x.value(), controller1.leftStick.y.value()), controller1.rightStick.x.value(), autoAlign, controller1.rightBumper.isPressed());
//
        controller1.update();
        controller2.update();
        drivetrain.update();
        poseEstimator.update();
        telemetry.addData("elevator position", elevator.getElevatorPosition());
        telemetry.addData("intake arm position", intake.getWristPos());
        telemetry.addData("x", drivetrain.getPose().getX(DistanceUnit.INCH));
        telemetry.addData("y", drivetrain.getPose().getY(DistanceUnit.INCH));
        telemetry.addData("yaw", Math.toDegrees(drivetrain.getPose().getHeading(AngleUnit.RADIANS)));
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