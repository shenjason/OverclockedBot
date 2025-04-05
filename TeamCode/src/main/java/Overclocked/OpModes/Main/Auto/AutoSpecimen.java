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
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import Overclocked.Constants.IntakeArmPose;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import Overclocked.Constants.IntakeSlidePose;
import Overclocked.Constants.OuttakeArmPose;
import Overclocked.Constants.OuttakeSlidePose;
import Overclocked.Subassemblies.Intake;
import Overclocked.Subassemblies.Outtake;

@Config
@Autonomous(name="AutoSpecimen", group = "Main", preselectTeleOp = "ManualControl")
public class AutoSpecimen extends OpMode {

    public static double SPEED = 1;


    private final double PREPICKUP_TO_PICKUP_TIME = 0;

    private final double PICKUP_CLOSECLAW_TIME = 0.3;

    private final double clipX = 44;
    private final double pushXStart = 54; private final double pushXEnd = 30;

    private final Pose start = new Pose(8, 64, Math.toRadians(180));
    private final Pose firstScore = new Pose(clipX, 72, Math.toRadians(180));
    private final Pose push1start = new Pose(pushXStart, 23.46, Math.toRadians(180));
    private final Pose push1end = new Pose(pushXEnd, push1start.getY(), Math.toRadians(180));
    private final Pose push2start = new Pose(pushXStart, 13.27, Math.toRadians(180));
    private final Pose push2end = new Pose(pushXEnd, push2start.getY(), Math.toRadians(180));
    private final Pose push3start = new Pose(pushXStart, 6.8, Math.toRadians(180));
    private final Pose push3end = new Pose(pushXEnd-5, push3start.getY(), Math.toRadians(180));

    private final Pose prePickupPose = new Pose(12, 34, Math.toRadians(180));
    private final Pose pickUpPose = new Pose(7, 34, Math.toRadians(180));

    private final Pose secondScore = new Pose(clipX + 3, 78, Math.toRadians(180));
    private final Pose thirdScore = new Pose(clipX + 3, 77.5, Math.toRadians(180));
    private final Pose forthScore = new Pose(clipX + 3, 77, Math.toRadians(180));
    private final Pose fifthScore =  new Pose(clipX + 3, 76.5, Math.toRadians(180));
    private final Pose park = new Pose(20.3, 30.8, Math.toRadians(0));

    //Timings

    private Follower follower;

    Intake intake;
    Outtake outtake;
    int pathState = 0;

    Timer pathTimer, opModeTimer;

    IMU imu;

    private PathChain firstScorePath, toPushPath, firstPushPath, secondPushPath, thirdPushPath, fromPushToScore, secondScorePath, thirdPickupPath, thirdScorePath, forthPickupPath, forthScorePath, fifthPickupPath, fifthScorePath, prePickupToPickUpPath, toPark;

    void buildPaths(){
        firstScorePath = follower.pathBuilder().addPath(new BezierLine(new Point(start), new Point(firstScore))).setLinearHeadingInterpolation(start.getHeading(), firstScore.getHeading()).build();

        toPushPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(firstScore), new Point(27.49, 68.83), new Point(10.38, 7.69), new Point(63.64, 47.102), new Point(push1start)))
                .setLinearHeadingInterpolation(firstScore.getHeading(), push1start.getHeading()).build();

//        toPushPath = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(40.5, 66), new Point(37, 66))).setConstantHeadingInterpolation(Math.toRadians(180))
//                .addPath(new BezierLine(new Point(37, 66), new Point(37, 35))).setConstantHeadingInterpolation(Math.toRadians(180))
//                .addPath(new BezierCurve(new Point(37, 35), new Point(60, 36), new Point(push1start))).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        firstPushPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(push1start), new Point(push1end)))
                .setLinearHeadingInterpolation(push1start.getHeading(), push1end.getHeading())
                .addPath(new BezierCurve(new Point(push1end), new Point(73, push1end.getY()), new Point(push2start)))
                .setLinearHeadingInterpolation(push1end.getHeading(), push2start.getHeading()).build();

        secondPushPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(push2start), new Point(push2end)))
                .setLinearHeadingInterpolation(push2start.getHeading(), push2end.getHeading())
                .addPath(new BezierCurve(new Point(push2end), new Point(73, push2end.getY()), new Point(push3start)))
                .setLinearHeadingInterpolation(push2end.getHeading(), push3start.getHeading()).build();

        thirdPushPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(push3start), new Point(push3end)))
                .setLinearHeadingInterpolation(push3start.getHeading(), push3end.getHeading()).build();

        fromPushToScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(push3end), new Point(prePickupPose)))
                .setLinearHeadingInterpolation(push3end.getHeading(), prePickupPose.getHeading()).build();

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

        toPark = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fifthScore), new Point(park))).setLinearHeadingInterpolation(fifthScore.getHeading(), park.getHeading()).build();
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
                if (follower.atParametricEnd() || pathTimer.getElapsedTimeSeconds() > 1.45){
                    outtake.pullDownSpecimen();
                    follower.followPath(toPushPath, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (follower.atParametricEnd()){
                    follower.followPath(firstPushPath, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (follower.atParametricEnd()){
                    follower.followPath(secondPushPath, true);

                    setPathState(4);
                }
                break;
            case 4:
                if (follower.atParametricEnd()){
                    follower.followPath(thirdPushPath, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (follower.atParametricEnd()){
                    outtake.setClawState(OuttakeArmPose.CLAW_OPEN);
                    follower.followPath(fromPushToScore, true);
                    setPathState(6);
                }
                break;

            // Second Score
            case 6:
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
                    follower.followPath(toPark);
                    setPathState(22);
                }
                break;
            case 22:
                break;
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


