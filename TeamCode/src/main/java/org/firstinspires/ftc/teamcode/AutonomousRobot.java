package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.math_utils.Vector;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.components.OpticalSensor;

import java.util.function.BooleanSupplier;

public class AutonomousRobot implements DriveConstants, ArmConstants {
    FtcDashboard dashboard;
    DriveTrain drivetrain;
    Elevator elevator;
    Intake intake;
    OpticalSensor od;
    Telemetry telemetry;


    public AutonomousRobot(boolean isBlue, HardwareMap hardwareMap, Telemetry telemetry) {
        Pose2D initialPose = new Pose2D(DistanceUnit.INCH, -24, -63, AngleUnit.DEGREES, 0);
        drivetrain = new DriveTrain(hardwareMap, initialPose, 90, false);
        intake = new Intake(hardwareMap, telemetry);
        elevator = new Elevator(hardwareMap);
        this.telemetry = telemetry;
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);
    }

    public void redAuto(BooleanSupplier isActive) {
        ElapsedTime time = new ElapsedTime();
        time.reset();
//go to bucket
        while (!elevator.elevatorToPos(ArmConstants.BUCKETHEIGHT) && isActive.getAsBoolean()) {
            drivetrain.update();
            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -51.5, -51.5, AngleUnit.DEGREES, 225), false);
            updateTelemetry();
        }

        time.reset();
//forward?
        while (time.seconds() < .7 && isActive.getAsBoolean()) {
            drivetrain.update();
            drivetrain.drive(new Vector(-.6, -.6), 0, false, false);
            updateTelemetry();
        }
// back up
        while (!drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -56, -56, AngleUnit.DEGREES, 225), false)) {
            drivetrain.update();
            updateTelemetry();

        }

//        while(!drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -24, -10, AngleUnit.DEGREES, 0),false) && isActive.getAsBoolean()){
//            drivetrain.update();
//            elevator.elevatorToPos(ArmConstants.ASCENTHEIGHT);
//
//            updateTelemetry();
//        }

//go to first block
        while (! elevator.elevatorToPos(ArmConstants.ELEVATORMIN) && isActive.getAsBoolean()) {
            drivetrain.update();
//            elevator.elevatorToPos(ArmConstants.ELEVATORMIN);
            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -21, -38  , AngleUnit.DEGREES, 345), false);
            //-26, -35.5
//            intake.intakeToPos(ArmConstants.INTAKEMAX);
            updateTelemetry();


        }
//intake out, servo spin, wrist go out
        while (!intake.intakeToPos(ArmConstants.INTAKEMAX) && isActive.getAsBoolean()) {
            updateTelemetry();
            intake.intakeServo(.8);
//            intake.wristControl(true);
        }
//intake servo run, drive a little forward
        time.reset();
        while (time.seconds() < 1.5 && isActive.getAsBoolean()) {
            drivetrain.update();
            intake.intakeServo(.8);
            drivetrain.drive(new Vector(-.4, .15), 0, false, true);
            updateTelemetry();
        }
//stop servos, intake in
        time.reset();
        intake.intakeServo(0);
//        intake.wristControl(false);
        while (!intake.intakeToPos(ArmConstants.INTAKEMIN) && isActive.getAsBoolean()) {
            updateTelemetry();
        }
//outtake block, and then stop

        time.reset();
        while (time.seconds() < .1 && isActive.getAsBoolean()) {

        }
        time.reset();
        while (time.seconds() < 1.1 && isActive.getAsBoolean()) {
            intake.outtakeBlock(.8);
            updateTelemetry();
        }
        intake.outtakeBlock(0);
        time.reset();
        while (time.seconds() < .1 && isActive.getAsBoolean()) {

        }
//        intake.wristControl(true);

//elevator up, go to bucket
        while (!elevator.elevatorToPos(ArmConstants.BUCKETHEIGHT) && isActive.getAsBoolean()) {
            drivetrain.update();
            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -53, -50, AngleUnit.DEGREES, 225), false);
            updateTelemetry();
        }
//push bucket
        time.reset();
        while (time.seconds() < .8 && isActive.getAsBoolean()) {
            drivetrain.update();
            drivetrain.drive(new Vector(-.6, -.6), 0, false, false);
            updateTelemetry();
        }
//back up, go down
        while (!drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -56, -56, AngleUnit.DEGREES, 225), false)) {
            drivetrain.update();
            updateTelemetry();



        }

//go to 2nd block
        while (!  elevator.elevatorToPos(ArmConstants.ELEVATORMIN) && isActive.getAsBoolean()) {
            drivetrain.update();
            elevator.elevatorToPos(ArmConstants.ELEVATORMIN);
            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -36, -35, AngleUnit.DEGREES, 350), false);
            //-26, -35.5
//            intake.intakeToPos(ArmConstants.INTAKEMAX);
            updateTelemetry();


        }
//intake out, spin servo
        while (!intake.intakeToPos(ArmConstants.INTAKEMAX) && isActive.getAsBoolean()) {
            updateTelemetry();
            intake.intakeServo(.8);
//            intake.wristControl(true);
        }

//spin intake, drive a lil forward
        time.reset();
        while (time.seconds() < 1.5 && isActive.getAsBoolean()) {
            drivetrain.update();
            intake.intakeServo(.8);
            drivetrain.drive(new Vector(-.4, .15), 0, false, false);
            updateTelemetry();
        }
//intake stop, intake back in
        time.reset();
        intake.intakeServo(0);
//        intake.wristControl(false);
        while (!intake.intakeToPos(ArmConstants.INTAKEMIN) && isActive.getAsBoolean()) {
            updateTelemetry();
        }
//outtake block
        time.reset();
        while (time.seconds() < .1 && isActive.getAsBoolean()) {

        }
        time.reset();
        while (time.seconds() < 1.1 && isActive.getAsBoolean()) {
            intake.outtakeBlock(.8);
            updateTelemetry();
        }

        intake.outtakeBlock(0);
        time.reset();
        while (time.seconds() < .1 && isActive. getAsBoolean()) {

        }
//        intake.wristControl(true);

//bucket time!!!
        while (!elevator.elevatorToPos(ArmConstants.BUCKETHEIGHT) && isActive.getAsBoolean()) {
            drivetrain.update();
            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -53, -50, AngleUnit.DEGREES, 225), false);
            updateTelemetry();
        }
//push in
        time.reset();
        while (time.seconds() < .8 && isActive.getAsBoolean()) {
            drivetrain.update();
            drivetrain.drive(new Vector(-.6, -.6), 0, false, false);
            updateTelemetry();
        }
//back up
        while (!elevator.elevatorToPos(ArmConstants.ELEVATORMIN)) {
            drivetrain.update();
            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -56, -56, AngleUnit.DEGREES, 225), false);
            updateTelemetry();
//            elevator.elevatorToPos(ArmConstants.ELEVATORMIN);

//        while(!drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -36, -20, AngleUnit.DEGREES, 90),false) && isActive.getAsBoolean()){
//            drivetrain.update();
//            updateTelemetry();
//
//        }
//
//        while(!drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -29, -16, AngleUnit.DEGREES, 90),false) && isActive.getAsBoolean()){
//            drivetrain.update();
//            updateTelemetry();
//
//
//        }
//
//        while(!elevator.elevatorToPos(ArmConstants.ASCENTHEIGHT) && isActive.getAsBoolean()){
//            updateTelemetry();
//        }


        }


    }

        public void updateTelemetry () {
            telemetry.addData("Robot Pose", drivetrain.getPose().toString());
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
            telemetry.addData("intake arm position", intake.getIntakePosition());
            telemetry.addData("intaketopose", Math.abs(intake.getIntakePosition() - 32000) < 1200);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }

