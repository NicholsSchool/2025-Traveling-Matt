package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.components.OpticalSensor;

public class AutonomousRobot implements DriveConstants, ArmConstants {
    private PoseEstimator poseEstimator;
    FtcDashboard dashboard;
    DriveTrain drivetrain;
    Elevator elevator;
    Intake intake;
    OpticalSensor od;
    private boolean isBlue;



    public AutonomousRobot(boolean isBlue, HardwareMap hardwareMap){
        drivetrain = new DriveTrain(hardwareMap, new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0), 0, false);
        intake = new Intake(hardwareMap, telemetry);
        elevator = new Elevator(hardwareMap);
        this.isBlue = isBlue;ElapsedTime time = new ElapsedTime();
        Pose2D initialPose = new Pose2D(DistanceUnit.INCH, -24, -63, AngleUnit.DEGREES, 90);
        poseEstimator = new PoseEstimator(hardwareMap, initialPose, true);
        dashboard = FtcDashboard.getInstance();
//        Telemetry dashboardTelemetry = dashboard.getTelemetry();
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//        telemetry.setMsTransmissionInterval(50);

    }

    public void redAuto(){
        drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -57, -57, AngleUnit.DEGREES, 225),false);
    }

    public void blueAuto(){


    }
}
