
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.math_utils.Vector;
import org.firstinspires.ftc.teamcode.subsystems.AutonomousRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

import java.util.concurrent.TimeUnit;

/**
 * Sample Auto to Copy Paste Edit with
 */
@Autonomous(name = "Contest Auto")
public class Auto extends LinearOpMode implements DriveConstants, ArmConstants {
    /**
     * Runs the Auto routine
     */
    @Override
    public void runOpMode() {
        AutonomousRobot robot = new AutonomousRobot(hardwareMap, 0, 0, 0);
        ElapsedTime time = new ElapsedTime();
        Elevator elevator = new Elevator(hardwareMap);
        DriveTrain drivetrain = new DriveTrain(hardwareMap, 0, 0, 3 * Math.PI / 4, false);
        waitForStart();
        telemetry.addData("elevator position", elevator.getElevatorPosition());
        telemetry.addData("heading", drivetrain.getHeading());
        telemetry.addData("x", drivetrain.getPose());

//
//        boolean pathOneIsFinished = false;
//        while(opModeIsActive() && !pathOneIsFinished)
//            pathOneIsFinished = robot.followPathOne();

        drivetrain.setTargetHeading(drivetrain.getPose().angle);

        time.reset();
        while (time.time(TimeUnit.SECONDS) < 3.8) {
            elevator.elevatorManual(1);
        }
        elevator.elevatorManual(0);
        time.reset();
        while (time.time(TimeUnit.SECONDS) < 1.0){
            drivetrain.drive(new Vector(12.0,12.0), 0, true);

            drivetrain.update();
            telemetry.update();

        }


        time.reset();
        while (time.time(TimeUnit.SECONDS) < 1.2) {
            drivetrain.drive(new Vector(-12.0, 12.0), 0, true);

            drivetrain.update();
            telemetry.update();
        }

        time.reset();
        while (time.time(TimeUnit.SECONDS) < 0.1) {
            drivetrain.drive(new Vector(3.0, -3.0), 0, true);

            drivetrain.update();
            telemetry.update();
        }
        time.reset();
        while (time.time(TimeUnit.SECONDS) < 4.2) {
            elevator.elevatorManual(-0.90);
            telemetry.update();
        }
        time.reset();
        while (time.time(TimeUnit.SECONDS) < 3.7) {
            drivetrain.drive(new Vector(0, -4), 0, true);
        }

        time.reset();
    }



 }


