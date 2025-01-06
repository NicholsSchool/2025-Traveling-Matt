package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.math_utils.PoseEstimator;

import java.util.Locale;

@TeleOp(name = "Pose Testing", group = "Dev")
public class PoseTesting extends OpMode {

    private PoseEstimator pose;


    @Override
    public void init() {

        pose = new PoseEstimator(hardwareMap, new Pose2D(DistanceUnit.INCH, 0, 60, AngleUnit.DEGREES, 0), true);

    }

    @Override
    public void loop() {

        pose.update();

        telemetry.addData("OTOS Heading", pose.otos.getHeading());
        telemetry.addData("OTOS Position", pose.otos.getPosition().toString());

        telemetry.addData("Initial Pose", String.format(Locale.US, "(%.3f, %.3f)", pose.initialPose.getX(DistanceUnit.INCH), pose.initialPose.getY(DistanceUnit.INCH)));
        telemetry.addData("Robot Pose", String.format(Locale.US, "(%.3f, %.3f)", pose.getPose().getX(DistanceUnit.INCH), pose.getPose().getY(DistanceUnit.INCH)));
        telemetry.addData("Using LL", pose.isUsingLL());

        telemetry.update();

    }
}