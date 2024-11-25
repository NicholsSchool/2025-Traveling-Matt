package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
//haven't made an ArmConstant Interface yet
//import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.controller.Controller;

import org.firstinspires.ftc.teamcode.math_utils.Vector;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

public class TeleopRobot implements DriveConstants, ArmConstants {

    private Controller driverOI;
    private Controller operatorOI;
    private Intake intake;
    private Outtake outtake;
    private DriveTrain drivetrain;

}
