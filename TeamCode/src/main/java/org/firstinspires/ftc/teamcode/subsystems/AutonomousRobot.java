package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.constants.SplineConstants;
import org.firstinspires.ftc.teamcode.math_utils.Angles;
import org.firstinspires.ftc.teamcode.math_utils.LerpPath;
import org.firstinspires.ftc.teamcode.math_utils.LerpPathPlanning;
import org.firstinspires.ftc.teamcode.math_utils.Point;
import org.firstinspires.ftc.teamcode.math_utils.Vector;


/**
 * Robot Container for the Autonomous Period
 */
public class AutonomousRobot {
    /**
     * The two target columns for autonomous
     */
    public enum TargetColumn {
        LEFT, RIGHT
    }

    private ElapsedTime timer;
    private DriveTrain drivetrain;
    private LerpPathPlanning lerpPathPlanning;

    /**
     * @param hardwareMap the hardware map
     * @param x the initial x
     * @param y the initial y
     * @param angle the initial angle
     */
    public AutonomousRobot(HardwareMap hardwareMap, double x, double y, double angle) {
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        drivetrain = new DriveTrain(hardwareMap, 0, 0, Math.PI / 2, false);

        LerpPath pathOne = new LerpPath(new Point(48.0, -24.0), Math.PI / 2);
        lerpPathPlanning = new LerpPathPlanning(drivetrain, new LerpPath[]{pathOne});

    }

    /**
     * Prepares the robot to follow the first auto path
     */
    public void prepForPathOne() {
    }


    public boolean followPathOne() {
        return lerpPathPlanning.spline(0.0, true, true);
    }
}