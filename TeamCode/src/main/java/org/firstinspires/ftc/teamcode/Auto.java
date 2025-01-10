
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.math_utils.PoseEstimator;
import org.firstinspires.ftc.teamcode.subsystems.AutonomousRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

/**
 * Sample Auto to Copy Paste Edit with
 */
@Autonomous(name = "Contest Auto")
public class Auto extends LinearOpMode implements DriveConstants, ArmConstants {
    private PoseEstimator poseEstimator;
    FtcDashboard dashboard;
    Controller driverOI;
    AutonomousRobot autonomousRobot;
    /**
     * Runs the Auto routine
     */
    @Override
    public void runOpMode() {

        ElapsedTime time = new ElapsedTime();
        Elevator elevator = new Elevator(hardwareMap);
        DriveTrain drivetrain = new DriveTrain(hardwareMap, new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0),0, false);
        Pose2D initialPose = new Pose2D(DistanceUnit.INCH, -24, -63, AngleUnit.DEGREES, 90);
        poseEstimator = new PoseEstimator(hardwareMap, initialPose, true);
        driverOI = new Controller(gamepad1);
        dashboard = FtcDashboard.getInstance();
        autonomousRobot = new AutonomousRobot(false, hardwareMap);
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);
        waitForStart();
        telemetry.addData("elevator position", elevator.getElevatorPosition());
        telemetry.addData("heading", drivetrain.getHeading());
        telemetry.addData("x", drivetrain.getPose());

//
//



        autonomousRobot.redAuto();
    }


 }


