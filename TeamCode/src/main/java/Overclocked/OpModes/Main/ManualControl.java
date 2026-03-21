package Overclocked.OpModes.Main;


import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Overclocked.Constants.IntakeArmPose;
import Overclocked.Constants.IntakeConstants;
import Overclocked.Constants.IntakeSlidePose;
import Overclocked.Constants.OuttakeArmPose;
import Overclocked.Constants.OuttakeConstants;
import Overclocked.Constants.OuttakeSlidePose;
import Overclocked.Subassemblies.Robot;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@TeleOp(name = "ManualControl", group = "Main")
public class ManualControl extends LinearOpMode {

    public Robot robot;
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);


    @Override
    public void runOpMode() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        robot = new Robot(hardwareMap, follower, OuttakeConstants.MODE_SAMPLE, IntakeConstants.SIDE_RED, false, gamepad1, gamepad2, telemetry);
        robot.outtake.waitForSlideReset();

        waitForStart();



        robot.outtake.setSlidePose(OuttakeSlidePose.INITIAL);
        robot.outtake.setOuttakeArmPose(OuttakeArmPose.TRANSFER);
        robot.intake.setArmPose(IntakeArmPose.TRANSFER);
        robot.intake.setSlidePose(IntakeSlidePose.FREE);

        while (opModeIsActive()) {

            robot.teleOpControl();
            robot.teleOpDebug();
            robot.update();
            
            telemetry.update();
        }
    }

}