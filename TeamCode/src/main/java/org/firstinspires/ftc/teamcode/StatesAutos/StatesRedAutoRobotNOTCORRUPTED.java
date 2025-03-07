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


public class StatesRedAutoRobotNOTCORRUPTED implements DriveConstants, ArmConstants {
    FtcDashboard dashboard;
    DriveTrain drivetrain;
    Elevator elevator;
    Intake intake;
    OpticalSensor od;
    Telemetry telemetry;
    ElapsedTime time = new ElapsedTime();


    public StatesRedAutoRobotNOTCORRUPTED(boolean isBlue, HardwareMap hardwareMap, Telemetry telemetry) {
        Pose2D initialPose = new Pose2D(DistanceUnit.INCH, -39, -63, AngleUnit.DEGREES, 225);
        drivetrain = new DriveTrain(hardwareMap, initialPose, 90, false);
        intake = new Intake(hardwareMap, telemetry);
        elevator = new Elevator(hardwareMap);
        this.telemetry = telemetry;
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);
        elevator.setState(Elevator.ELEVATOR_STATE.GO_TO_POSITION);
    }

    public void redAuto(BooleanSupplier isActive) {
        time.reset();

//go to bucket #0
        while (!elevator.elevatorToPos(ArmConstants.BUCKETHEIGHT) && isActive.getAsBoolean()) {
            drivetrain.update();
            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -51.5, -50, AngleUnit.DEGREES, 225), false);
            updateTelemetry();
            intake.periodic();
            elevator.periodic();
        }

        time.reset();
//forward #0
        while (time.seconds() < .7 && isActive.getAsBoolean()) {
            drivetrain.update();
            drivetrain.drive(new Vector(-.6, -.6), 0, false, false);
            updateTelemetry();
            elevator.periodic();
            intake.periodic();
        }
// back up #0
        while (!drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -57.5, -55, AngleUnit.DEGREES, 225), false)) {
            drivetrain.update();
            elevator.periodic();
            updateTelemetry();
            intake.periodic();

        }
//midpoint #1
        while (!drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -48, -54, AngleUnit.DEGREES, 270), false) && isActive.getAsBoolean()) {
            drivetrain.update();
            elevator.elevatorToPos(ArmConstants.ASCENTHEIGHT);
            elevator.periodic();
                intake.periodic();

            updateTelemetry();
        }

//go to #1 block
        while (!drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -44.5, -42.2, AngleUnit.DEGREES, 267), false) && isActive.getAsBoolean()) {
            drivetrain.update();
//            elevator.elevatorToPos(ArmConstants.ELEVATORMIN);
//            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -43.5, -56  , AngleUnit.DEGREES, 267), false);
//            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -44.5, -30  , AngleUnit.DEGREES, 267), false);
            elevator.elevatorToPos(ArmConstants.ELEVATORMIN);
            elevator.periodic();
            //-26, -35.5
//            intake.intakeToPos(ArmConstants.INTAKEMAX);
            updateTelemetry();


        }
//stop motors running #1
        drivetrain.runDriveMotors(0);

        //intake and run slides out #1
        while (!intake.setWristSetpoint(85)) {
            elevator.periodic();
            intake.periodic();
            updateTelemetry();
            intake.intakeServo(.8);
            intake.periodic();
            intake.intakeToPos(16700);
            intake.periodic();
            telemetry.addLine("Im in the loop");
            telemetry.addLine("Im updated");
            intake.periodic();

        }

        //intake block #1
        time.reset();
        while (time.seconds() < 0.6 && isActive.getAsBoolean() || intake.hasSample() && isActive.getAsBoolean()) {
            intake.intakeServo(.8);
        }

        intake.intakeServo(0);
        time.reset();
        intake.intakeServo(0);
        //go to handoff #1
        while (!intake.setWristSetpoint(ArmConstants.INTAKE_BUCKET) && isActive.getAsBoolean()) {
            intake.periodic();
            elevator.periodic();

            intake.intakeToPos(ArmConstants.INTAKEMIN);
            updateTelemetry();
        }

        intake.intakeSoftLimited(0);

//outtake block #1

        time.reset();
        while (time.seconds() < .1 && isActive.getAsBoolean()) {

        }
        time.reset();
        while (time.seconds() < 1.0 && isActive.getAsBoolean()) {
            intake.outtakeBlock(.5);
            updateTelemetry();
        }
        intake.outtakeBlock(0);
        time.reset();
        while (time.seconds() < .1 && isActive.getAsBoolean()) {

        }

        while (!intake.setWristSetpoint(250)) {
            intake.periodic();
            updateTelemetry();
            intake.periodic();
        }

//go to bucket #1
        while (!elevator.elevatorToPos(ArmConstants.BUCKETHEIGHT) && isActive.getAsBoolean()) {
            drivetrain.update();
            elevator.periodic();
            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -51.5, -50, AngleUnit.DEGREES, 225), false);
            updateTelemetry();
            intake.periodic();
        }

        time.reset();
//forward #1
        while (time.seconds() < .7 && isActive.getAsBoolean()) {
            drivetrain.update();
            elevator.periodic();
            drivetrain.drive(new Vector(-.6, -.6), 0, false, false);
            updateTelemetry();
            intake.periodic();
        }
// back up #1
        while (!drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -57.5, -55, AngleUnit.DEGREES, 225), false)) {
            drivetrain.update();
            updateTelemetry();
            elevator.periodic();
            intake.periodic();

        }

        // midpoint #2
        while (!drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -52, -54, AngleUnit.DEGREES, 270), false) && isActive.getAsBoolean()) {
            drivetrain.update();
            elevator.periodic();
            elevator.elevatorToPos(ArmConstants.ASCENTHEIGHT);
            intake.periodic();

            updateTelemetry();
        }

//go to #2 block
        while (!drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -54.5, -41.0, AngleUnit.DEGREES, 267), false) && isActive.getAsBoolean()) {
            drivetrain.update();
            elevator.periodic();
//            elevator.elevatorToPos(ArmConstants.ELEVATORMIN);
//            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -43.5, -56  , AngleUnit.DEGREES, 267), false);
//            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -44.5, -30  , AngleUnit.DEGREES, 267), false);
            elevator.elevatorToPos(ArmConstants.ELEVATORMIN);
            //-26, -35.5
//            intake.intakeToPos(ArmConstants.INTAKEMAX);
            updateTelemetry();


        }

        drivetrain.runDriveMotors(0);
        intake.intakeSoftLimited(0);

        //wrist/intake/slides go out #2
        while (!intake.setWristSetpoint(85)) {
            elevator.periodic();
            intake.periodic();
            updateTelemetry();
            intake.intakeServo(.8);
            intake.periodic();
            intake.intakeToPos(16600);
            intake.periodic();
            telemetry.addLine("Im in the loop");
            telemetry.addLine("Im updated");
            intake.periodic();

        }

        //intake run #2
        time.reset();
        while (time.seconds() < 0.7 && isActive.getAsBoolean()) {
            intake.intakeServo(.8);
        }

        //intake handoff #2
        time.reset();
        intake.intakeServo(0);
        while (!intake.setWristSetpoint(ArmConstants.INTAKE_BUCKET) && isActive.getAsBoolean()) {
            intake.periodic();

            intake.intakeToPos(ArmConstants.INTAKEMIN);
            updateTelemetry();
        }

//outtake block #2
        time.reset();
        while (time.seconds() < .1 && isActive.getAsBoolean()) {

        }
        time.reset();
        while (time.seconds() < 1.0 && isActive.getAsBoolean()) {
            intake.outtakeBlock(.5);
            updateTelemetry();
        }
        intake.outtakeBlock(0);
        time.reset();
        while (time.seconds() < .1 && isActive.getAsBoolean()) {

        }

        while (!intake.setWristSetpoint(250)) {
            intake.periodic();
            updateTelemetry();
            intake.periodic();
        }

//bucket time!!! #2
        while (!elevator.elevatorToPos(ArmConstants.BUCKETHEIGHT) && isActive.getAsBoolean()) {
            elevator.periodic();
            drivetrain.update();
            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -51.5, -50, AngleUnit.DEGREES, 225), false);
            updateTelemetry();
            intake.periodic();
        }
//push in #2
        time.reset();
        while (time.seconds() < .7 && isActive.getAsBoolean()) {
            drivetrain.update();
            elevator.periodic();
            drivetrain.drive(new Vector(-.6, -.6), 0, false, false);
            updateTelemetry();
            intake.periodic();
        }
//back up #2
        while (!drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -57.5, -55, AngleUnit.DEGREES, 225), false)) {
            drivetrain.update();
            elevator.periodic();
            updateTelemetry();
            intake.periodic();

        }


        // midpoint #3
        while (!drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -52, -52, AngleUnit.DEGREES, 270), false) && isActive.getAsBoolean()) {
            drivetrain.update();
            elevator.periodic();
            elevator.elevatorToPos(ArmConstants.ASCENTHEIGHT);
            intake.periodic();

            updateTelemetry();
        }

        // go to #3 block
        while (!drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -52.2, -43, AngleUnit.DEGREES, 300), false) && isActive.getAsBoolean()) {
            drivetrain.update();

            elevator.periodic();

            elevator.elevatorToPos(ArmConstants.ELEVATORMIN);

            updateTelemetry();


        }

        drivetrain.runDriveMotors(0);

        //run wrist/slides/intake out #3
        while (!intake.setWristSetpoint(145 )) {
            intake.periodic();
            elevator.periodic();
            updateTelemetry();
            intake.intakeServo(.8);
            intake.periodic();
            intake.intakeToPos(22550);
            intake.periodic();
            telemetry.addLine("Im in the loop");
            telemetry.addLine("Im updated");
            intake.periodic();

        }

        //intake #3
        time.reset();
        while (time.seconds() < 0.7 && isActive.getAsBoolean()) {
            intake.intakeServo(.8);
        }
//intake servo run, drive a little forward


        //|| intake.hasSample() && isActive.getAsBoolean()
// intake in
        time.reset();
        intake.intakeServo(0);
        while (!intake.setWristSetpoint(ArmConstants.INTAKE_BUCKET) && isActive.getAsBoolean()) {
            intake.periodic();
            elevator.periodic();

            intake.intakeToPos(ArmConstants.INTAKEMIN);
            updateTelemetry();
        }

        intake.intakeSoftLimited(0);
//outtake block, and then stop

        time.reset();
        while (time.seconds() < .1 && isActive.getAsBoolean()) {

        }
        time.reset();
        while (time.seconds() < 1.0 && isActive.getAsBoolean()) {
            intake.outtakeBlock(.6);
            updateTelemetry();
        }
        intake.outtakeBlock(0);
        time.reset();
        while (time.seconds() < .1 && isActive.getAsBoolean()) {

        }

        while (!intake.setWristSetpoint(250)) {
            intake.periodic();
            elevator.periodic();
            updateTelemetry();
            intake.periodic();
        }

//go to bucket #3
        while (!elevator.elevatorToPos(ArmConstants.BUCKETHEIGHT) && isActive.getAsBoolean()) {
            drivetrain.update();
            elevator.periodic();
            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -51.5, -50, AngleUnit.DEGREES, 225), false);
            updateTelemetry();
            intake.periodic();
        }
//push in #2
        time.reset();
        while (time.seconds() < .7 && isActive.getAsBoolean()) {
            drivetrain.update();
            elevator.periodic();
            drivetrain.drive(new Vector(-.6, -.6), 0, false, false);
            updateTelemetry();
            intake.periodic();
        }
//back up #2
        while (!drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -57.5, -55, AngleUnit.DEGREES, 225), false)) {
            drivetrain.update();
            elevator.periodic();
            updateTelemetry();
            intake.periodic();

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
        telemetry.addData("intake arm position", intake.getIntakeSlidePosition());
        telemetry.addData("wrist postion", intake.getWristPos());
        telemetry.addData("time", time.seconds());
//            telemetry.addData("intakeToPose?", intake.intakeToPos(ArmConstants.INTAKEMAX));
//            telemetry.addData("intake at pos?", Math.abs(intake.getWristPos() - ArmConstants.INTAKEMAX) < 1200);
        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }
}



