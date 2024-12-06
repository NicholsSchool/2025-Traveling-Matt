
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.math_utils.LerpPath;
import org.firstinspires.ftc.teamcode.math_utils.LerpPathPlanning;
import org.firstinspires.ftc.teamcode.math_utils.Point;
import org.firstinspires.ftc.teamcode.subsystems.AutonomousRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

/**
 * Sample Auto to Copy Paste Edit with
 */
@Autonomous(name = "Blue Auto")
public class Auto extends LinearOpMode implements DriveConstants, ArmConstants {
    /**
     * Runs the Auto routine
     */
    @Override
    public void runOpMode() {
        AutonomousRobot robot = new AutonomousRobot(hardwareMap ,0 ,0,0);
        waitForStart();

        boolean pathOneIsFinished = false;
        while(opModeIsActive() && !pathOneIsFinished)
            pathOneIsFinished = robot.followPathOne();

        terminateOpModeNow();
    }
}
