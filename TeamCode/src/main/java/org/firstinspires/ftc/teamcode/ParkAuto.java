
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.math_utils.PoseEstimator;

/**
 * Sample Auto to Copy Paste Edit with
 */
@Autonomous(name = "parkAuto")
public class ParkAuto extends LinearOpMode implements DriveConstants, ArmConstants {
    private PoseEstimator poseEstimator;
    FtcDashboard dashboard;
    Controller driverOI;
    ParkAuto parkAuto;
    ParkAutonomousRobot parkAutonomousRobot;
    /**
     * Runs the Auto routine
     */
    @Override
    public void runOpMode() {


        ElapsedTime time = new ElapsedTime();
//        DriveTrain drivetrain = new DriveTrain(hardwareMap, 0, 0, 0, false);

//        driverOI = new Controller(gamepad1);
        parkAutonomousRobot = new ParkAutonomousRobot(false, hardwareMap, telemetry);
        waitForStart();

//
//


        parkAutonomousRobot.parkAuto(this::opModeIsActive);



    }


}


