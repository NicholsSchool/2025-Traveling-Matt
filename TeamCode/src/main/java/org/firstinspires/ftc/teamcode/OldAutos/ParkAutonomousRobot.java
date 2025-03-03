package org.firstinspires.ftc.teamcode.OldAutos;

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

public class ParkAutonomousRobot implements DriveConstants, ArmConstants {
    FtcDashboard dashboard;
    DriveTrain drivetrain;
    Elevator elevator;
    Intake intake;
    OpticalSensor od;
    Telemetry telemetry;



    public ParkAutonomousRobot(boolean isBlue, HardwareMap hardwareMap, Telemetry telemetry){
        Pose2D initialPose = new Pose2D(DistanceUnit.INCH, -24, -63, AngleUnit.DEGREES, 0);
        drivetrain = new DriveTrain(hardwareMap, initialPose, 90, false);
        intake = new Intake(hardwareMap, telemetry);
        elevator = new Elevator(hardwareMap);
        this.telemetry = telemetry;
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);
    }

    public void parkAuto(BooleanSupplier isActive){
        ElapsedTime time = new ElapsedTime();
        time.reset();

        while(!elevator.elevatorToPos(ArmConstants.BUCKETHEIGHT) && isActive.getAsBoolean()){
            drivetrain.update();
            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -52, -52, AngleUnit.DEGREES, 225),false);
            updateTelemetry();
        }

        time.reset();

        while(time.seconds() < 1 && isActive.getAsBoolean()) {
            drivetrain.update();
            drivetrain.drive(new Vector(-2, -2), 0, false, false);
            updateTelemetry();
        }

        while(!drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -56, -56, AngleUnit.DEGREES, 225),false)){
            drivetrain.update();
            updateTelemetry();
        }

        while(!drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -24, -10, AngleUnit.DEGREES, 0),false) && isActive.getAsBoolean()){
            drivetrain.update();
            elevator.elevatorToPos(ArmConstants.ASCENTHEIGHT);

            updateTelemetry();
        }



//
////        intake.intakeServo(.8);
////        intake.wristControl(true);
////        while(!intake.intakeToPos(ArmConstants.INTAKEMAX) && isActive.getAsBoolean()){
////            updateTelemetry();
////        }
////
////        while(!drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -26, -35.5, AngleUnit.DEGREES, 320),false) && isActive.getAsBoolean()){
////            drivetrain.update();
////            //-26, -35.5
//////            intake.intakeToPos(ArmConstants.INTAKEMAX);
////            updateTelemetry();
////            intake.intakeServo(.8);
////
////        }
////        time.reset();
////        while(time.seconds() < 1.5 && isActive.getAsBoolean()) {
////            intake.intakeServo(.8);
////            updateTelemetry();
////        }
////
////
////        intake.intakeServo(0);
////        intake.wristControl(false);
////        while(!intake.intakeToPos(ArmConstants.INTAKEMIN) && isActive.getAsBoolean()){
////            updateTelemetry();
////        }
////        intake.outtakeBlock(.3);
//
////        while(!drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -36, -20, AngleUnit.DEGREES, 90),false) && isActive.getAsBoolean()){
////            drivetrain.update();
////            updateTelemetry();
////
////        }
////
////        while(!drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -29, -16, AngleUnit.DEGREES, 90),false) && isActive.getAsBoolean()){
////            drivetrain.update();
////            updateTelemetry();
////
////
////        }
////
////        while(!elevator.elevatorToPos(ArmConstants.ASCENTHEIGHT) && isActive.getAsBoolean()){
////            updateTelemetry();
////        }
//
//
//
//
//
//
//
//
//
 }

    public void blueAuto(BooleanSupplier isActive){
        ElapsedTime time = new ElapsedTime();
        time.reset();

        while(!elevator.elevatorToPos(ArmConstants.BUCKETHEIGHT) && isActive.getAsBoolean()){
            drivetrain.update();
            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -57, -57, AngleUnit.DEGREES, 225),false);
            updateTelemetry();
        }

        time.reset();

        while(time.seconds() < 1 && isActive.getAsBoolean()) {
            drivetrain.update();
            drivetrain.drive(new Vector(-2, -2), 0, false, false);
            updateTelemetry();
        }

        while(!drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -56, -56, AngleUnit.DEGREES, 225),false)){
            drivetrain.update();
            updateTelemetry();
        }

        while(!elevator.elevatorToPos(ArmConstants.ELEVATORMIN) && isActive.getAsBoolean()){
            drivetrain.update();
            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -24, -37.5, AngleUnit.DEGREES, 90),false);
            updateTelemetry();
        }


    }

    public void updateTelemetry() {
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
        telemetry.addData("intake arm position", intake.getWristPos());
        telemetry.addData("intaketopose", Math.abs(intake.getWristPos() - 32000) < 1200);
        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }
}
