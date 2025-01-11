package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.CompTeleop;
import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.math_utils.PoseEstimator;
import org.firstinspires.ftc.teamcode.math_utils.Vector;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.components.OpticalSensor;

import java.util.function.BooleanSupplier;

public class AutonomousRobot implements DriveConstants, ArmConstants {
    FtcDashboard dashboard;
    DriveTrain drivetrain;
    Elevator elevator;
    Intake intake;
    OpticalSensor od;
    Telemetry telemetry;



    public AutonomousRobot(boolean isBlue, HardwareMap hardwareMap, Telemetry telemetry){
        Pose2D initialPose = new Pose2D(DistanceUnit.INCH, -24, -63, AngleUnit.DEGREES, 0);
        drivetrain = new DriveTrain(hardwareMap, initialPose, 90, false);
        intake = new Intake(hardwareMap, telemetry);
        elevator = new Elevator(hardwareMap);
        this.telemetry = telemetry;
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);
    }

    public void redAuto(BooleanSupplier isActive){
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
            drivetrain.drive(new Vector(-.4, -.4), 0, false, false);
            updateTelemetry();
        }

        while(!drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -56, -56, AngleUnit.DEGREES, 225),false)){
            drivetrain.update();
            updateTelemetry();
        }

        while(!elevator.elevatorToPos(ArmConstants.ELEVATORMIN) && isActive.getAsBoolean()){
            drivetrain.update();
            drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -24, -37.5, AngleUnit.DEGREES, 340),false);
            updateTelemetry();
        }

        intake.intakeServo(.8);
        intake.wristControl(true);
        while(!intake.intakeToPos(ArmConstants.INTAKEMAX) && isActive.getAsBoolean()){
            updateTelemetry();
        }


        while(!drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -26, -35.5, AngleUnit.DEGREES, 340),false) && isActive.getAsBoolean()){
            drivetrain.update();
            updateTelemetry();
        }

        intake.intakeServo(0);
        intake.wristControl(false);
        while(!intake.intakeToPos(ArmConstants.INTAKEMIN) && isActive.getAsBoolean()){
            updateTelemetry();
        }
        intake.outtakeBlock(.3);








    }

    public void blueAuto(){


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
        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }
}
