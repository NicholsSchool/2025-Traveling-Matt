
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
@Autonomous(name = "blueAuto")
public class BlueAuto extends LinearOpMode implements DriveConstants, ArmConstants {
    private PoseEstimator poseEstimator;
    FtcDashboard dashboard;
    Controller driverOI;
    BlueAutonomousRobot blueautonomousrobot;
    /**
     * Runs the Auto routine
     */
    @Override
    public void runOpMode() {

        ElapsedTime time = new ElapsedTime();
//        DriveTrain drivetrain = new DriveTrain(hardwareMap, 0, 0, 0, false);

//        driverOI = new Controller(gamepad1);
       blueautonomousrobot = new BlueAutonomousRobot(false, hardwareMap, telemetry);
        waitForStart();

//
//


        blueautonomousrobot.blueAuto(this::opModeIsActive);



    }


}


