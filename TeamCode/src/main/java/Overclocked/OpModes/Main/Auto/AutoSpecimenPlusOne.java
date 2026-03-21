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
import Overclocked.Constants.IntakeSlidePose;
import Overclocked.Constants.OuttakeArmPose;
import Overclocked.Constants.OuttakeSlidePose;
import Overclocked.Subassemblies.Intake;
import Overclocked.Subassemblies.Outtake;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name="AutoSpecimenPlusOne", group = "Main", preselectTeleOp = "ManualControl")
public class AutoSpecimenPlusOne extends OpMode {

    public static double SPEED = 1;

    public static int firstSweepTicks = 280;
    public static int secondSweepTicks = 400;
    public static int thirdSweepTicks = 320;

    public static double firstSweepStartX = 40.95;
    public static double firstSweepStartY = 38.3;
    public static double firstSweepEndX = 28;
    public static double firstSweepEndY = 34.6;
    public static double secondSweepStartX = 36;
    public static double secondSweepStartY = 28;
    public static double secondSweepEndX = 23.83;
    public static double secondSweepEndY = 27.68;

    public static double thirdSweepStartX = 35.57;
    public static double thirdSweepStartY = 20;
    public static double thirdSweepEndX = 23.84;
    public static double thirdSweepEndY = 20.19;



    private final double PREPICKUP_TO_PICKUP_TIME = 0;

    private final double PICKUP_CLOSECLAW_TIME = 0.25;

    private final double clipX = 43;

    private final Pose start = new Pose(8, 64, Math.toRadians(180));
    private final Pose firstScore = new Pose(clipX, 69, Math.toRadians(180));

    private Pose firstSweepStart = new Pose(firstSweepStartX, firstSweepStartY, Math.toRadians(-45));
    private Pose firstSweepEnd = new Pose(firstSweepEndX, firstSweepEndY, Math.toRadians(-135));
    private Pose secondSweepStart = new Pose(secondSweepStartX, secondSweepStartY, Math.toRadians(-20));
    private Pose secondSweepEnd = new Pose(secondSweepEndX, secondSweepEndY, Math.toRadians(-145));
    private Pose thirdSweepStart = new Pose(thirdSweepStartX, thirdSweepStartY, Math.toRadians(-40));
    private Pose thirdSweepEnd = new Pose(thirdSweepEndX, thirdSweepEndY, Math.toRadians(-145));

    private final Pose prePickupPose = new Pose(12, 32, Math.toRadians(180));
    private final Pose pickUpPose = new Pose(7, 32, Math.toRadians(180));

    private final Pose secondScore = new Pose(clipX + 3, 78, Math.toRadians(180));
    private final Pose thirdScore = new Pose(clipX + 3, 76, Math.toRadians(180));
    private final Pose forthScore = new Pose(clipX + 3, 74, Math.toRadians(180));
    private final Pose fifthScore =  new Pose(clipX + 3, 72, Math.toRadians(180));
    private final Pose plusOneScore = new Pose(6.73, 126.31, Math.toRadians(-90));

    //Timings

    private Follower follower;

    Intake intake;
    Outtake outtake;
    int pathState = 0;

    Timer pathTimer, opModeTimer;

    IMU imu;

    private PathChain firstScorePath, toPushPath, firstPushPath, secondPushPath, thirdPushPath, fromPushToScore, secondScorePath, thirdPickupPath, thirdScorePath, forthPickupPath, forthScorePath, fifthPickupPath, fifthScorePath, prePickupToPickUpPath, toPlusOnePickup, toPlusOne;

    void buildPaths(){
        firstScorePath = follower.pathBuilder().addPath(new BezierLine(new Point(start), new Point(firstScore))).setLinearHeadingInterpolation(start.getHeading(), firstScore.getHeading()).build();

        toPushPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(firstScore), new Point(7.11, 36), new Point(firstSweepStart)))
                .setLinearHeadingInterpolation(firstScore.getHeading(), firstSweepStart.getHeading()).build();

        firstPushPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstSweepStart), new Point(firstSweepEnd)))
                .setLinearHeadingInterpolation(firstSweepStart.getHeading(), firstSweepEnd.getHeading())
                .addPath(new BezierLine(new Point(firstSweepEnd), new Point(secondSweepStart)))
                .setLinearHeadingInterpolation(firstSweepEnd.getHeading(), secondSweepStart.getHeading()).build();


        secondPushPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondSweepStart), new Point(secondSweepEnd)))
                .setLinearHeadingInterpolation(secondSweepStart.getHeading(), secondSweepEnd.getHeading())
                .addPath(new BezierLine(new Point(secondSweepEnd), new Point(thirdSweepStart)))
                .setLinearHeadingInterpolation(secondSweepEnd.getHeading(), thirdSweepStart.getHeading()).build();

        thirdPushPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdSweepStart), new Point(thirdSweepEnd)))
                .setLinearHeadingInterpolation(thirdSweepStart.getHeading(), thirdSweepEnd.getHeading()).build();

        fromPushToScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdSweepEnd), new Point(prePickupPose)))
                .setLinearHeadingInterpolation(thirdSweepEnd.getHeading(), prePickupPose.getHeading()).build();

        secondScorePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpPose), new Point(secondScore)))
                .setLinearHeadingInterpolation(pickUpPose.getHeading(), secondScore.getHeading()).build();
        thirdScorePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpPose), new Point(thirdScore)))
                .setLinearHeadingInterpolation(pickUpPose.getHeading(), thirdScore.getHeading()).build();

        forthScorePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpPose), new Point(forthScore)))
                .setLinearHeadingInterpolation(pickUpPose.getHeading(), forthScore.getHeading()).build();
        fifthScorePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpPose), new Point(fifthScore)))
                .setLinearHeadingInterpolation(pickUpPose.getHeading(), fifthScore.getHeading()).build();


        thirdPickupPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondScore), new Point(prePickupPose)))
                .setLinearHeadingInterpolation(secondScore.getHeading(), prePickupPose.getHeading()).build();
        forthPickupPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdScore), new Point(prePickupPose)))
                .setLinearHeadingInterpolation(secondScore.getHeading(), prePickupPose.getHeading()).build();
        fifthPickupPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(forthScore), new Point(prePickupPose)))
                .setLinearHeadingInterpolation(secondScore.getHeading(), prePickupPose.getHeading()).build();


        prePickupToPickUpPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prePickupPose), new Point(pickUpPose)))
                .setLinearHeadingInterpolation(prePickupPose.getHeading(), pickUpPose.getHeading()).setZeroPowerAccelerationMultiplier(2).build();

        toPlusOnePickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fifthScore), new Point(prePickupPose))).setLinearHeadingInterpolation(fifthScore.getHeading(), prePickupPose.getHeading())
                .build();


        toPlusOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fifthScore), new Point(plusOneScore))).setLinearHeadingInterpolation(fifthScore.getHeading(), plusOneScore.getHeading()).build();
    }



    public void autoStateUpdate(){
        switch (pathState){
            case 0:
                outtake.setSlidePose(OuttakeSlidePose.SPECIMEN_SCORE);
                outtake.setOuttakeArmPose(OuttakeArmPose.SPECIMEN_SCORE);
                follower.followPath(firstScorePath, true);
                setPathState(1);
                break;
            case 1:
                outtake.setSlidePose(OuttakeSlidePose.SPECIMEN_SCORE);
                if (follower.atParametricEnd() || pathTimer.getElapsedTimeSeconds() > 1.6){
                    outtake.pullDownSpecimen();
                    follower.followPath(toPushPath, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 0.6){
                    outtake.setOuttakeArmPose(OuttakeArmPose.TRANSFER);
                    intake.autoSetSlidePos(firstSweepTicks);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.8){
                    intake.setArmPose(IntakeArmPose.SWEEP);
                }
                if (follower.atParametricEnd()){
                    follower.followPath(firstPushPath, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 0.8){
                    intake.autoSetSlidePos(0);
                }
                if (follower.atParametricEnd()){
                    intake.autoSetSlidePos(secondSweepTicks);
                    follower.followPath(secondPushPath, true);

                    setPathState(4);
                }
                break;
            case 4:
                if (pathTimer.getElapsedTimeSeconds() > 0.8){
                    intake.autoSetSlidePos(0);
                }
                if (follower.atParametricEnd()){
                    intake.autoSetSlidePos(thirdSweepTicks);
                    follower.followPath(thirdPushPath, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTimeSeconds() > 0.8){
                    intake.autoSetSlidePos(0);
                }
                if (follower.atParametricEnd()){
                    intake.setArmPose(IntakeArmPose.INACTIVE);
                    intake.setSlidePose(IntakeSlidePose.INITIAL);
                    follower.followPath(fromPushToScore, true);
                    setPathState(6);
                }
                break;

            // Second Score
            case 6:
                if (pathTimer.getElapsedTimeSeconds() > 0.5){
                    outtake.setOuttakeArmPose(OuttakeArmPose.SPECIMEN_PICKUP);
                }
                if (follower.atParametricEnd()){
                    setPathState(7);
                }
                break;

            case 7:
                if (pathTimer.getElapsedTimeSeconds() > PREPICKUP_TO_PICKUP_TIME){
                    follower.followPath(prePickupToPickUpPath , true);
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTimeSeconds() > PICKUP_CLOSECLAW_TIME){
                    outtake.setClawState(OuttakeArmPose.CLAW_CLOSE);
                }
                if (pathTimer.getElapsedTimeSeconds() > PICKUP_CLOSECLAW_TIME + 0.05){
                    outtake.setSlidePose(OuttakeSlidePose.SPECIMEN_SCORE);
                    outtake.setOuttakeArmPose(OuttakeArmPose.SPECIMEN_SCORE);
                }
                if (pathTimer.getElapsedTimeSeconds() > PICKUP_CLOSECLAW_TIME + 0.45){
                    follower.followPath(secondScorePath, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (follower.atParametricEnd()  || pathTimer.getElapsedTimeSeconds() > 1.8){
                    outtake.setSlidePose(OuttakeSlidePose.INITIAL);
                    outtake.setOuttakeArmPose(OuttakeArmPose.SPECIMEN_PICKUP);
                    follower.followPath(thirdPickupPath, true);
                    setPathState(10);
                }
                break;

            // Third Score

            case 10:
                if (follower.atParametricEnd() || pathTimer.getElapsedTimeSeconds() > 1.8){
                    setPathState(11);
                }
                break;

            case 11:
                if (pathTimer.getElapsedTimeSeconds() > PREPICKUP_TO_PICKUP_TIME){
                    follower.followPath(prePickupToPickUpPath , true);
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds() > PICKUP_CLOSECLAW_TIME){
                    outtake.setClawState(OuttakeArmPose.CLAW_CLOSE);
                }
                if (pathTimer.getElapsedTimeSeconds() > PICKUP_CLOSECLAW_TIME + 0.05){
                    outtake.setSlidePose(OuttakeSlidePose.SPECIMEN_SCORE);
                    outtake.setOuttakeArmPose(OuttakeArmPose.SPECIMEN_SCORE);
                }
                if (pathTimer.getElapsedTimeSeconds() > PICKUP_CLOSECLAW_TIME + 0.45){
                    follower.followPath(thirdScorePath, true);
                    setPathState(13);
                }
                break;
            case 13:
                if (follower.atParametricEnd() || pathTimer.getElapsedTimeSeconds() > 1.8){
                    outtake.setSlidePose(OuttakeSlidePose.INITIAL);
                    outtake.setOuttakeArmPose(OuttakeArmPose.SPECIMEN_PICKUP);
                    follower.followPath(forthPickupPath, true);
                    setPathState(14);
                }
                break;

            // Forth Score

            case 14:
                if (follower.atParametricEnd() || pathTimer.getElapsedTimeSeconds() > 1.8){
                    setPathState(15);
                }
                break;

            case 15:
                if (pathTimer.getElapsedTimeSeconds() > PREPICKUP_TO_PICKUP_TIME){
                    follower.followPath(prePickupToPickUpPath, true);
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTimeSeconds() > PICKUP_CLOSECLAW_TIME){
                    outtake.setClawState(OuttakeArmPose.CLAW_CLOSE);
                }
                if (pathTimer.getElapsedTimeSeconds() > PICKUP_CLOSECLAW_TIME + 0.05){
                    outtake.setSlidePose(OuttakeSlidePose.SPECIMEN_SCORE);
                    outtake.setOuttakeArmPose(OuttakeArmPose.SPECIMEN_SCORE);
                }
                if (pathTimer.getElapsedTimeSeconds() > PICKUP_CLOSECLAW_TIME + 0.45){
                    follower.followPath(forthScorePath, true);
                    setPathState(17);
                }
                break;
            case 17:
                if (follower.atParametricEnd()  || pathTimer.getElapsedTimeSeconds() > 1.8){
                    outtake.setSlidePose(OuttakeSlidePose.INITIAL);
                    outtake.setOuttakeArmPose(OuttakeArmPose.SPECIMEN_PICKUP);
                    follower.followPath(fifthPickupPath, true);
                    setPathState(18);
                }
                break;
            case 18:
                if (follower.atParametricEnd() || pathTimer.getElapsedTimeSeconds() > 1.8){
                    setPathState(19);
                }
                break;

            //Fifth score

            case 19:
                if (pathTimer.getElapsedTimeSeconds() > PREPICKUP_TO_PICKUP_TIME){
                    follower.followPath(prePickupToPickUpPath , true);
                    setPathState(20);
                }
                break;
            case 20:
                if (pathTimer.getElapsedTimeSeconds() > PICKUP_CLOSECLAW_TIME){
                    outtake.setClawState(OuttakeArmPose.CLAW_CLOSE);
                }
                if (pathTimer.getElapsedTimeSeconds() > PICKUP_CLOSECLAW_TIME + 0.05){
                    outtake.setSlidePose(OuttakeSlidePose.SPECIMEN_SCORE);
                    outtake.setOuttakeArmPose(OuttakeArmPose.SPECIMEN_SCORE);
                }
                if (pathTimer.getElapsedTimeSeconds() > PICKUP_CLOSECLAW_TIME + 0.45){
                    follower.followPath(fifthScorePath, true);
                    setPathState(21);
                }
                break;
            case 21:
                if (follower.atParametricEnd()  || pathTimer.getElapsedTimeSeconds() > 1.8){
                    outtake.setSlidePose(OuttakeSlidePose.INITIAL);
                    outtake.setOuttakeArmPose(OuttakeArmPose.SPECIMEN_PICKUP);
                    follower.followPath(toPlusOnePickup);
                    setPathState(22);
                }
                break;
            //Plus One

            case 22:
                if (follower.atParametricEnd() || pathTimer.getElapsedTimeSeconds() > 1.8){
                    setPathState(23);
                }
                break;
            case 23:
                if (pathTimer.getElapsedTimeSeconds() > PREPICKUP_TO_PICKUP_TIME){
                    follower.followPath(prePickupToPickUpPath , true);
                    setPathState(24);
                }
                break;
            case 24:
                if (pathTimer.getElapsedTimeSeconds() > PICKUP_CLOSECLAW_TIME){
                    outtake.setClawState(OuttakeArmPose.CLAW_CLOSE);
                }
                if (pathTimer.getElapsedTimeSeconds() > PICKUP_CLOSECLAW_TIME + 0.05){
                    outtake.setSlidePose(OuttakeSlidePose.SWITCH);
                }
                if (pathTimer.getElapsedTimeSeconds() > PICKUP_CLOSECLAW_TIME + 0.45){
                    follower.followPath(toPlusOne, true);
                    setPathState(25);
                }
                break;
            case 25:
                if (pathTimer.getElapsedTimeSeconds() > 1.5){
                    outtake.setSlidePose(OuttakeSlidePose.SAMPLE_SCORE);
                    outtake.setOuttakeArmPose(OuttakeArmPose.SAMPLE_SCORE);
                }
                if (follower.atParametricEnd()){
                    outtake.setClawState(OuttakeArmPose.CLAW_OPEN);
                    setPathState(26);
                }
               break;
            case 26:
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    outtake.setSlidePose(OuttakeSlidePose.INITIAL);
                    outtake.setOuttakeArmPose(OuttakeArmPose.TRANSFER);
                    setPathState(-1);
                }
        }
    }

    public void subassembiliesInit(){
        outtake = new Outtake(hardwareMap, false);
        intake = new Intake(hardwareMap);

        intake.setSlidePose(IntakeSlidePose.INITIAL);

        outtake.setSlidePose(OuttakeSlidePose.INITIAL);
        outtake.setOuttakeArmPose(OuttakeArmPose.INITIAL_AUTO);
        outtake.setClawState(OuttakeArmPose.CLAW_CLOSE);
    }

    public void setPathState(int state){
        pathState = state;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {

        subassembiliesInit();

        opModeTimer = new Timer();
        pathTimer = new Timer();

        imu = hardwareMap.get(IMU.class, "imu");

        imu.resetDeviceConfigurationForOpMode();


        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

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
        intake.update(telemetry);
        outtake.update(telemetry);


        follower.update();
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


