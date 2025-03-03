package org.firstinspires.ftc.teamcode.StatesAutos;

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

public class StatesRedAutoRobot implements DriveConstants, ArmConstants {
    FtcDashboard dashboard;
    DriveTrain drivetrain;
    Elevator elevator;
    Intake intake;
    OpticalSensor od;
    Telemetry telemetry;
    ElapsedTime time = new ElapsedTime();


    public StatesRedAutoRobot(boolean isBlue, HardwareMap hardwareMap, Telemetry telemetry) {
        Pose2D initialPose = new Pose2D(DistanceUnit.INCH, -39, -63, AngleUnit.DEGREES, 225);
        drivetrain = new DriveTrain(hardwareMap, initialPose, 90, false);
        intake = new Intake(hardwareMap, telemetry);
        elevator = new Elevator(hardwareMap);
        this.telemetry = telemetry;
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);
    }

    public void redAuto(BooleanSupplier isActive) {
        time.reset();
//go to bucket
        while (!elevator.elevatorToPos(ArmConstants.BUCKETHEIGHT) && isActive.getAsBoolean()) {
            drivetrain.update();
            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -51.5, -50, AngleUnit.DEGREES, 225), false);
            updateTelemetry();
            intake.periodic();
        }

        time.reset();
//forward?
        while (time.seconds() < .7 && isActive.getAsBoolean()) {
            drivetrain.update();
            drivetrain.drive(new Vector(-.6, -.6), 0, false, false);
            updateTelemetry();
            intake.periodic();
        }
// back up
        while (!drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -57.5, -55, AngleUnit.DEGREES, 225), false)) {
            drivetrain.update();
            updateTelemetry();
            intake.periodic();

        }
//
        while(!drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -48, -54, AngleUnit.DEGREES, 270),false) && isActive.getAsBoolean()){
            drivetrain.update();
            elevator.elevatorToPos(ArmConstants.ASCENTHEIGHT);
            intake.periodic();

            updateTelemetry();
        }

//go to first block
        while (! elevator.elevatorToPos(ArmConstants.ELEVATORMIN) && isActive.getAsBoolean()) {
            drivetrain.update();
//            elevator.elevatorToPos(ArmConstants.ELEVATORMIN);
//            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -43.5, -56  , AngleUnit.DEGREES, 267), false);
            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -43.5, -45  , AngleUnit.DEGREES, 267), false);
            //-26, -35.5
//            intake.intakeToPos(ArmConstants.INTAKEMAX);
            updateTelemetry();


        }
//intake out, servo spin, wrist go out
//        while (!intake.setWristSetpoint(160)) {
//            intake.periodic();
//            updateTelemetry();
//            intake.intakeServo(.8);
//            intake.periodic();
//            intake.intakeToPos(ArmConstants.INTAKEMAX);
//            intake.periodic();
//            telemetry.addLine("Im in the loop");
//            intake.periodic();
//
//        }

        while (!intake.setWristSetpoint(100)) {
            intake.periodic();
            updateTelemetry();
            intake.intakeServo(.8);
            intake.periodic();
            intake.intakeToPos(14500);
            intake.periodic();
            telemetry.addLine("Im in the loop");
            intake.periodic();

        }
//intake servo run, drive a little forward
        time.reset();
        while (time.seconds() < 1.87 && isActive.getAsBoolean()) {
            intake.periodic();
            drivetrain.update();
            intake.periodic();
//            drivetrain.drive(new Vector(0, .4), 0, false, false);
            intake.periodic();
            intake.intakeServo(.6);
            intake.periodic();
            updateTelemetry();
            intake.periodic();
            drivetrain.update();
        }

        //|| intake.hasSample() && isActive.getAsBoolean()
//stop servos, intake in
        time.reset();
        intake.intakeServo(0);
//        intake.setIntakeSetpoint(ArmConstants.INTAKE_BUCKET);
//        intake.wristControl(false);
        while (!intake.intakeToPos(ArmConstants.INTAKEMIN) && isActive.getAsBoolean()) {
            intake.periodic();
            intake.setWristSetpoint(ArmConstants.INTAKE_BUCKET);
//            if (intake.getIntakeSlidePosition() > -12000) {
//                intake.setIntakeSetpoint(ArmConstants.INTAKE_BUCKET);
//            } else {
//                intake.setIntakeSetpoint(intake.getIntakeSlidePosition() * ArmConstants.LINEAR_REGRESSION_M + ArmConstants.LINEAR_REGRESSION_B);
//            }
//            updateTelemetry();
        }
////outtake block, and then stop
//
//        time.reset();
//        while (time.seconds() < .1 && isActive.getAsBoolean()) {
//
//        }
//        time.reset();
//        while (time.seconds() < 1.1 && isActive.getAsBoolean()) {
//            intake.outtakeBlock(.8);
//            updateTelemetry();
//        }
//        intake.outtakeBlock(0);
//        time.reset();
//        while (time.seconds() < .1 && isActive.getAsBoolean()) {
//
//        }
////        intake.wristControl(true);
//
////elevator up, go to bucket
//        while (!elevator.elevatorToPos(ArmConstants.BUCKETHEIGHT) && isActive.getAsBoolean()) {
//            drivetrain.update();
//            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -53, -50, AngleUnit.DEGREES, 225), false);
//            updateTelemetry();
//        }
////push bucket
//        time.reset();
//        while (time.seconds() < .8 && isActive.getAsBoolean()) {
//            drivetrain.update();
//            drivetrain.drive(new Vector(-.6, -.6), 0, false, false);
//            updateTelemetry();
//        }
////back up, go down
//        while (!drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -56, -56, AngleUnit.DEGREES, 225), false)) {
//            drivetrain.update();
//            updateTelemetry();
//
//
//
//        }
//
////go to 2nd block
//        while (!  elevator.elevatorToPos(ArmConstants.ELEVATORMIN) && isActive.getAsBoolean()) {
//            drivetrain.update();
//            elevator.elevatorToPos(ArmConstants.ELEVATORMIN);
//            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -36, -35, AngleUnit.DEGREES, 350), false);
//            //-26, -35.5
////            intake.intakeToPos(ArmConstants.INTAKEMAX);
//            updateTelemetry();
//
//
//        }
////intake out, spin servo
//        while (!intake.intakeToPos(ArmConstants.INTAKEMAX) && isActive.getAsBoolean()) {
//            updateTelemetry();
//            intake.intakeServo(.8);
////            intake.wristControl(true);
//        }
//
////spin intake, drive a lil forward
//        time.reset();
//        while (time.seconds() < 1.5 && isActive.getAsBoolean()) {
//            drivetrain.update();
//            intake.intakeServo(.8);
//            drivetrain.drive(new Vector(-.4, .15), 0, false, false);
//            updateTelemetry();
//        }
////intake stop, intake back in
//        time.reset();
//        intake.intakeServo(0);
////        intake.wristControl(false);
//        while (!intake.intakeToPos(ArmConstants.INTAKEMIN) && isActive.getAsBoolean()) {
//            updateTelemetry();
//        }
////outtake block
//        time.reset();
//        while (time.seconds() < .1 && isActive.getAsBoolean()) {
//
//        }
//        time.reset();
//        while (time.seconds() < 1.1 && isActive.getAsBoolean()) {
//            intake.outtakeBlock(.8);
//            updateTelemetry();
//        }
//
//        intake.outtakeBlock(0);
//        time.reset();
//        while (time.seconds() < .1 && isActive. getAsBoolean()) {
//
//        }
////        intake.wristControl(true);
//
////bucket time!!!
//        while (!elevator.elevatorToPos(ArmConstants.BUCKETHEIGHT) && isActive.getAsBoolean()) {
//            drivetrain.update();
//            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -53, -50, AngleUnit.DEGREES, 225), false);
//            updateTelemetry();
//        }
////push in
//        time.reset();
//        while (time.seconds() < .8 && isActive.getAsBoolean()) {
//            drivetrain.update();
//            drivetrain.drive(new Vector(-.6, -.6), 0, false, false);
//            updateTelemetry();
//        }
////back up
//        while (!elevator.elevatorToPos(ArmConstants.ELEVATORMIN)) {
//            drivetrain.update();
//            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -56, -56, AngleUnit.DEGREES, 225), false);
//            updateTelemetry();
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

//
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
            telemetry.addData("intake arm position", intake.getIntakeSlidePosition());
            telemetry.addData("wrist postion", intake.getWristPos());
            telemetry.addData("time", time.seconds());
//            telemetry.addData("intakeToPose?", intake.intakeToPos(ArmConstants.INTAKEMAX));
//            telemetry.addData("intake at pos?", Math.abs(intake.getWristPos() - ArmConstants.INTAKEMAX) < 1200);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }



