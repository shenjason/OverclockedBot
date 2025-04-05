package Overclocked.OpModes.Main.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import Overclocked.Constants.IntakeArmPose;
import Overclocked.Constants.IntakeConstants;
import Overclocked.Constants.IntakeSlidePose;
import Overclocked.Constants.OuttakeArmPose;
import Overclocked.Constants.OuttakeConstants;
import Overclocked.Constants.OuttakeSlidePose;
import Overclocked.Subassemblies.Robot;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name="AutoSample", group = "Main", preselectTeleOp = "ManualControl")
public class AutoSample extends OpMode {

    public static double SPEED = 1;

    public static int PickupSlidePos = 258;
    public static int LatPickupSlidePos = 250;

    private final Pose start = new Pose(7, 113, Math.toRadians(0));
    private final Pose scorePose = new Pose(15,  125.6, Math.toRadians(-45));
    private final Pose secondPickup = new Pose(24.2, 121, Math.toRadians(0));
    private final Pose thirdPickup = new Pose(24.2, 129.2, Math.toRadians(0));
    private final Pose forthPickup = new Pose(35, 121, Math.toRadians(90));
    private final Pose park = new Pose(72, 94, Math.toRadians(-90));

    //Timings

    private Follower follower;

    Robot robot;
    int pathState = 0;

    Timer pathTimer, opModeTimer;

    IMU imu;

    private PathChain secondPickupPath, thirdPickupPath, forthPickupPath, firstScorePath, secondScorePath, thirdScorePath, forthScorePath, toPark;

    void buildPaths(){
        firstScorePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(start), new Point(scorePose)))
                .setLinearHeadingInterpolation(start.getHeading(), scorePose.getHeading()).build();
        secondPickupPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(secondPickup)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), secondPickup.getHeading()).build();
        secondScorePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondPickup), new Point(scorePose)))
                .setLinearHeadingInterpolation(secondPickup.getHeading(), scorePose.getHeading()).build();
        thirdPickupPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(thirdPickup)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), thirdPickup.getHeading()).build();
        thirdScorePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdPickup), new Point(scorePose)))
                .setLinearHeadingInterpolation(thirdPickup.getHeading(), scorePose.getHeading()).build();
        forthPickupPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(forthPickup)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), forthPickup.getHeading()).build();
        forthScorePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(forthPickup), new Point(scorePose)))
                .setLinearHeadingInterpolation(forthPickup.getHeading(), scorePose.getHeading()).build();

        toPark = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(park)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), park.getHeading()).build();
    }



    public void autoStateUpdate(){
        switch (pathState){
            case 0:
                robot.outtake.setSlidePose(OuttakeSlidePose.SAMPLE_SCORE);
                robot.outtake.setOuttakeArmPose(OuttakeArmPose.SAMPLE_SCORE);
                follower.followPath(firstScorePath, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && robot.outtake.canChangeSlidePoseTimed()){
                    robot.outtake.setClawState(OuttakeArmPose.CLAW_OPEN);
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    robot.outtake.setSlidePose(OuttakeSlidePose.INITIAL);
                    robot.outtake.setOuttakeArmPose(OuttakeArmPose.TRANSFER);
                    robot.intake.autoSetSlidePos(PickupSlidePos);
                    follower.followPath(secondPickupPath);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    robot.intake.setArmPose(IntakeArmPose.DETECTION);
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.8){
                    robot.intake.pickup();
                    setPathState(4);
                }
                break;
            case 4:
                if (pathTimer.getElapsedTimeSeconds() > 0.5){
                    robot.transfer();
                    setPathState(5);
                }
                break;
            case 5:
                if (!robot.isInTransfer()){
                    robot.outtake.setSlidePose(OuttakeSlidePose.SAMPLE_SCORE);
                    robot.outtake.setOuttakeArmPose(OuttakeArmPose.SAMPLE_SCORE);
                    follower.followPath(secondScorePath);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() && robot.outtake.canChangeSlidePoseTimed()){
                    robot.outtake.setClawState(OuttakeArmPose.CLAW_OPEN);
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    robot.outtake.setSlidePose(OuttakeSlidePose.INITIAL);
                    robot.outtake.setOuttakeArmPose(OuttakeArmPose.TRANSFER);
                    robot.intake.autoSetSlidePos(PickupSlidePos);
                    follower.followPath(thirdPickupPath);
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    robot.intake.setArmPose(IntakeArmPose.DETECTION);
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.8){
                    robot.intake.pickup();
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTimeSeconds() > 0.5){
                    robot.transfer();
                    setPathState(10);
                }
                break;
            case 10:
                if (!robot.isInTransfer()){
                    robot.outtake.setSlidePose(OuttakeSlidePose.SAMPLE_SCORE);
                    robot.outtake.setOuttakeArmPose(OuttakeArmPose.SAMPLE_SCORE);
                    follower.followPath(thirdScorePath);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy() && robot.outtake.canChangeSlidePoseTimed()){
                    robot.outtake.setClawState(OuttakeArmPose.CLAW_OPEN);
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    robot.outtake.setSlidePose(OuttakeSlidePose.INITIAL);
                    robot.outtake.setOuttakeArmPose(OuttakeArmPose.TRANSFER);
                    robot.intake.autoSetSlidePos(LatPickupSlidePos);
                    follower.followPath(forthPickupPath);
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    robot.intake.setArmPose(IntakeArmPose.DETECTION);
                    robot.intake.setOverride_auto_rotation(4);
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.95){
                    robot.intake.setOverride_auto_rotation(4);
                    robot.intake.pickup();
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTimeSeconds() > 0.5){
                    robot.transfer();
                    setPathState(15);
                }
                break;
            case 15:
                if (!robot.isInTransfer()){
                    robot.outtake.setSlidePose(OuttakeSlidePose.SAMPLE_SCORE);
                    robot.outtake.setOuttakeArmPose(OuttakeArmPose.SAMPLE_SCORE);
                    follower.followPath(forthScorePath);
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy() && robot.outtake.canChangeSlidePoseTimed()){
                    robot.outtake.setClawState(OuttakeArmPose.CLAW_OPEN);
                    setPathState(17);
                }
                break;
            case 17:
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    robot.outtake.setSlidePose(OuttakeSlidePose.INITIAL);
                    robot.outtake.setOuttakeArmPose(OuttakeArmPose.TRANSFER);
                    robot.intake.setSlidePose(IntakeSlidePose.INITIAL);
                    robot.intake.setSlidePose(IntakeArmPose.INACTIVE);
                    follower.followPath(toPark);
                    setPathState(-1);
                }
                break;
        }
    }

    public void subassembiliesInit(){
        robot = new Robot(hardwareMap, follower, OuttakeConstants.MODE_SAMPLE, IntakeConstants.SIDE_RED, true, null, null, telemetry);

        robot.intake.setSlidePose(IntakeSlidePose.INITIAL);

        robot.outtake.setSlidePose(OuttakeSlidePose.INITIAL);
        robot.outtake.setOuttakeArmPose(OuttakeArmPose.INITIAL_AUTO);
        robot.outtake.setClawState(OuttakeArmPose.CLAW_CLOSE);
    }

    public void setPathState(int state){
        pathState = state;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {

        opModeTimer = new Timer();
        pathTimer = new Timer();

        imu = hardwareMap.get(IMU.class, "imu");

        imu.resetDeviceConfigurationForOpMode();


        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        subassembiliesInit();

        imu.resetYaw();

        follower.setStartingPose(start);
        follower.setMaxPower(SPEED);

        buildPaths();
    }


    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        robot.update();

        follower.drawOnDashBoard();

        autoStateUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {
    }

    @Override
    public void init_loop() {}
}


