
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

import java.util.function.BooleanSupplier;

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
//        DriveTrain drivetrain = new DriveTrain(hardwareMap, 0, 0, 0, false);

//        driverOI = new Controller(gamepad1);
        autonomousRobot = new AutonomousRobot(false, hardwareMap, telemetry);
        waitForStart();

//
//


        autonomousRobot.redAuto(this::opModeIsActive);



    }


 }


