package org.firstinspires.ftc.teamcode;

//haven't made an ArmConstant Interface yet
//import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.controller.Controller;

        import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

public class TeleopRobot implements DriveConstants, ArmConstants {

    private Controller driverOI;
    private Controller operatorOI;
    private Intake intake;
    private Elevator outtake;
    private DriveTrain drivetrain;

}
