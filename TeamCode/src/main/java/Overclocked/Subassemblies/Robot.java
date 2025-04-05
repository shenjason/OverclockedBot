package Overclocked.Subassemblies;


import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;


import Overclocked.Constants.IntakeArmPose;
import Overclocked.Constants.IntakeConstants;
import Overclocked.Constants.IntakeSlidePose;
import Overclocked.Constants.OuttakeArmPose;
import Overclocked.Constants.OuttakeConstants;
import Overclocked.Constants.OuttakeSlidePose;
import Overclocked.Util.ActionPress;

public class Robot {

    public final double HEADING_SENSITIVITY = 0.8;

    public IMU imu;
    public Intake intake;
    public Outtake outtake;
    public Follower follower;

    public CameraDetection cameraDetection;
    private Telemetry t;
    private Gamepad g1, g2;

    private boolean isAuto;

    private Timer transferTimer, modeSwitchTimer, autoAngleTimer, autoTuneCooldown, p2pPathTimer;
    private boolean inTransfer1, inTransfer2, switching, SIDE, MODE, driveLocked, autoTune, autoPickUp, inP2P;

    public double VOLTAGE, driveTrainSpeed, maxDriveTrainSpeed, Y_SPEED, X_SPEED, H_SPEED, lockedAngle;

    public ActionPress outtakeCloseClaw, intakeCloseClaw, intakePickUp, switchOuttakeStateAction, modeSwitch, transferAction, p2pAction, lowBasketAction;






    public Robot(HardwareMap hardwareMap, Follower follower, boolean startMode, boolean Side, boolean isAuto, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry){
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap, !isAuto);
        cameraDetection = new CameraDetection(hardwareMap, telemetry, true, Side);
        this.follower = follower;
        driveLocked = false;
        t = telemetry;
        g1 = gamepad1;
        g2 = gamepad2;
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));


        this.MODE = startMode;
        this.SIDE = Side;


        transferTimer = new Timer();
        modeSwitchTimer = new Timer();
        autoAngleTimer = new Timer();
        autoTuneCooldown = new Timer();
        p2pPathTimer = new Timer();

        this.isAuto = isAuto;

        if (!isAuto){
            outtakeCloseClaw = new ActionPress(() -> outtake.switchClawState());
            intakeCloseClaw = new ActionPress(() -> intake.switchClawState());
            intakePickUp = new ActionPress(() -> intake.pickup());
            transferAction = new ActionPress(this::transfer);
            switchOuttakeStateAction = new ActionPress(this::switchOuttakeState);
            modeSwitch = new ActionPress(this::switchMode);
            p2pAction = new ActionPress(this::P2P);
            lowBasketAction = new ActionPress(this::lowBasket);


            follower.startTeleopDrive();
        }
    }

    public void teleOpControl(){

        driveTrainControls();

        outtakeCloseClaw.update((g2.right_trigger > 0));
        intakeCloseClaw.update((g2.left_trigger > 0));
        intakePickUp.update(g2.left_bumper, intake.isArmPose(IntakeArmPose.DETECTION) && !(inTransfer1 || inTransfer2));
        transferAction.update(g2.a, intake.isArmPose(IntakeArmPose.DETECTION));
        switchOuttakeStateAction.update(g2.right_bumper, outtake.canChangeSlidePoseTimed());
        lowBasketAction.update(g2.b, outtake.canChangeSlidePoseTimed());
        modeSwitch.update(g2.dpad_up, outtake.isSlidePose(OuttakeSlidePose.INITIAL));
        p2pAction.update(g1.a, MODE == OuttakeConstants.MODE_SPECIMEN);


        if (inP2P && p2pPathTimer.getElapsedTimeSeconds() > 4.2){
            follower.breakFollowing();
            follower.startTeleopDrive();
            g1.rumble(100);
            inP2P = false;
        }

        if (inP2P && g1.right_bumper || g1.left_bumper){
            follower.breakFollowing();
            follower.startTeleopDrive();
            g1.rumble(100);
            inP2P = false;
        }

        //IntakeSlideControl
        double slideSpeed = 0.4;
        if (g2.y) slideSpeed = 1;

        if (intake.isSlidePose(IntakeSlidePose.FREE)) intake.slideControl(-g2.left_stick_y * slideSpeed);

        //ManualOverrideWristPosition
        if (g2.right_stick_y < -0.5){
            intake.setOverride_auto_rotation(4);
        } else if (g2.right_stick_x < -0.5) {
            intake.setOverride_auto_rotation(2);
        }else if (g2.right_stick_x > 0.5){
                intake.setOverride_auto_rotation(3);
        }else{
            intake.setOverride_auto_rotation(1);
        }

        //Autotune (Get it? Auto-tune)
        autoTune = g2.right_stick_button && intake.isArmPose(IntakeArmPose.DETECTION) && (autoTuneCooldown.getElapsedTimeSeconds() > 0.5);

        if ((autoTune && cameraDetection.hasDetection)){
            if (!autoPickUp){
                if (!driveLocked) {
                    g1.rumble(100);
                    imu.resetYaw();
                }
                driveLocked = true;
                //P Tune ClawPos
                intake.setSlidePose(IntakeSlidePose.AUTO_PICKUP);

//            if (cameraDetection.angle != 1000) intake.setWristMidAngle(cameraDetection.angle);

                if (Math.abs(cameraDetection.ErrorX) > 5){
                    int correction_dir = -(int)Math.signum(cameraDetection.ErrorX);
                    intake.slideControl(Math.max(IntakeConstants.MIN_SLIDE_CORRECTION_POWER, Math.min(IntakeConstants.MAX_SLIDE_CORRECTION_POWER, Math.abs(cameraDetection.ErrorX) * IntakeConstants.SLIDE_CORRECTION_P)) * correction_dir);
                }else intake.slideControl(0);

                if (Math.abs(cameraDetection.ErrorY) > 8){
                    int correction_dir = (int)Math.signum(cameraDetection.ErrorY);

                    follower.setTeleOpMovementVectors(0, Math.max(IntakeConstants.MIN_DRIVE_CORRECTION_POWER, Math.min(IntakeConstants.MAX_DRIVE_CORRECTION_POWER, Math.abs(cameraDetection.ErrorY) * IntakeConstants.DRIVE_CORRECTION_P)) * correction_dir, angleDiff(imu.getRobotYawPitchRollAngles().getYaw()) * -0.1, true);
                }else follower.setTeleOpMovementVectors(0, 0, angleDiff(imu.getRobotYawPitchRollAngles().getYaw()) * -0.1, true);

                if (Math.abs(cameraDetection.ErrorY) <= 8 && Math.abs(cameraDetection.ErrorX) <= 5){
                    autoPickUp = true;
                    lockedAngle = cameraDetection.angle;
                    autoAngleTimer.resetTimer();
                }
            }

        }else{
            if (driveLocked) g1.rumble(50);
            driveLocked = false;
            if (intake.isSlidePose(IntakeSlidePose.AUTO_PICKUP)) intake.setSlidePose(IntakeSlidePose.FREE);
        }


        //AutoPickup
        if (autoPickUp){
            driveLocked = true;
            intake.setOverride_auto_rotation(0);
            intake.setWristMidAngle(lockedAngle);

            if (autoAngleTimer.getElapsedTimeSeconds() > 0.25){
                intake.pickup();
                g1.rumble(30);
                g2.rumble(30);
                autoTuneCooldown.resetTimer();
                autoPickUp = false;
            }
        }

    }


    public void driveTrainControls(){
        maxDriveTrainSpeed = 1;
        if (VOLTAGE > 13.5) maxDriveTrainSpeed = 0.9;
        if (VOLTAGE < 12.5){
            maxDriveTrainSpeed = 0.8;
            if (VOLTAGE < 10.5){
                maxDriveTrainSpeed = 0.6;
            }
        }


        driveTrainSpeed = (MODE == OuttakeConstants.MODE_SAMPLE) ? 1 : 0.7;
        if (g1.left_bumper || g1.right_bumper){
            driveTrainSpeed = 0.3f;
        }

        int X_DIR = (int)-Math.signum(g1.left_stick_y);
        int Y_DIR = (int)-Math.signum(g1.left_stick_x);
        int H_DIR = (int)-Math.signum(g1.right_stick_x);

        X_SPEED = Math.min(Math.pow(Math.abs(g1.left_stick_y), 3) * driveTrainSpeed, maxDriveTrainSpeed);
        Y_SPEED = Math.min(Math.pow(Math.abs(g1.left_stick_x), 3) * driveTrainSpeed, maxDriveTrainSpeed);
        H_SPEED = Math.min(Math.abs(g1.right_stick_x) * HEADING_SENSITIVITY * driveTrainSpeed, maxDriveTrainSpeed);


        if (!driveLocked) follower.setTeleOpMovementVectors(X_SPEED * X_DIR,  Y_SPEED * Y_DIR, H_SPEED * H_DIR, true);
    }

    public void teleOpDebug(){
        this.t.addLine("ROBOT");

        this.t.addData("Mode", MODE);
        this.t.addData("Side", SIDE);
        this.t.addData("Voltage", VOLTAGE);
        this.t.addData("DriveSpeed", maxDriveTrainSpeed);
        this.t.addData("ForwardSpeed", X_SPEED);
        this.t.addData("StrafeSpeed", Y_SPEED);
        this.t.addData("HeadingSpeed", H_SPEED);
        this.t.addData("HeadingRaw", imu.getRobotYawPitchRollAngles().getYaw());

        this.t.addLine("FOLLOWER");

        this.t.addData("RobotX", follower.getPose().getX());
        this.t.addData("RobotY", follower.getPose().getY());
        this.t.addData("RobotHeading", follower.getPose().getHeading());
    }


    public void switchOuttakeState(){
        if (outtake.isSlidePose(OuttakeSlidePose.INITIAL)){

            if (MODE){
                outtake.setSlidePose(OuttakeSlidePose.SAMPLE_SCORE);
                outtake.setOuttakeArmPose(OuttakeArmPose.SAMPLE_SCORE);
            }else{
                outtake.setSlidePose(OuttakeSlidePose.SPECIMEN_SCORE);
                outtake.setOuttakeArmPose(OuttakeArmPose.SPECIMEN_SCORE);
            }

        }else{

            if (MODE){
                outtake.setSlidePose(OuttakeSlidePose.INITIAL);
                outtake.setOuttakeArmPose(OuttakeArmPose.TRANSFER);
            }else{
                outtake.pullDownSpecimen();
            }

        }
    }
    public void lowBasket(){
        if (!MODE) return;
        if (outtake.isSlidePose(OuttakeSlidePose.INITIAL)){
            outtake.setSlidePose(OuttakeSlidePose.LOW_BASKET);
            outtake.setOuttakeArmPose(OuttakeArmPose.SAMPLE_SCORE);
        }else{
            outtake.setSlidePose(OuttakeSlidePose.INITIAL);
            outtake.setOuttakeArmPose(OuttakeArmPose.TRANSFER);
        }
    }




    public void switchMode(){
        g1.rumble(200);
        g2.rumble(200);
        MODE = !MODE;
        if (MODE == OuttakeConstants.MODE_SAMPLE){
            outtake.setOuttakeArmPose(OuttakeArmPose.TRANSFER);
            intake.setSlidePose(IntakeSlidePose.FREE);
        }
        else {
            outtake.setOuttakeArmPose(OuttakeArmPose.SPECIMEN_PICKUP);
            outtake.setSlidePose(OuttakeSlidePose.SWITCH);
            intake.setArmPose(IntakeArmPose.INACTIVE);
            intake.setSlidePose(IntakeSlidePose.INITIAL);
            modeSwitchTimer.resetTimer();
            switching = true;
        }
    }

    public boolean isInTransfer(){
        return inTransfer1 || inTransfer2;
    }




    public void update(){
        intake.update(this.t);
        outtake.update(this.t);

        cameraDetection.update();


        if (modeSwitchTimer.getElapsedTimeSeconds() > 0.6 && switching){
            outtake.setSlidePose(OuttakeSlidePose.INITIAL);
            switching = false;
        }

        if (inTransfer1 && transferTimer.getElapsedTimeSeconds() > 0.4){
            intake.setSlidePose(IntakeSlidePose.TRANSFER);
            intake.setOverride_auto_rotation(1);
            if (intake.isSlideAtInitial()){
                inTransfer2 = true;
                inTransfer1 = false;
                transferTimer.resetTimer();
            }
        }

        if (inTransfer2){
            outtake.setClawState(OuttakeArmPose.CLAW_CLOSE);
            if (transferTimer.getElapsedTimeSeconds() > 0.25){
                intake.setClawState(IntakeArmPose.CLAW_OPEN);
                intake.setSlidePose(IntakeSlidePose.FREE);
                inTransfer2 = false;
                if (!isAuto) g2.rumble(100);
            }

        }

        VOLTAGE = follower.getVoltage();

        follower.update();
    }

    public void transfer(){
        inTransfer1 = true;
        intake.setArmPose(IntakeArmPose.TRANSFER);
        outtake.setClawState(OuttakeArmPose.CLAW_OPEN);
        transferTimer.resetTimer();

    }


    public void P2P(){
        follower.setPose(new Pose(0, 0, Math.toRadians(180)));
        PathChain p2pPath = follower.pathBuilder().
                addPath(new BezierCurve(new Point(0, 0), new Point( 6, 0), new Point(29, 40), new Point(35, 40))).setPathEndTimeoutConstraint(0).setConstantHeadingInterpolation(Math.toRadians(180))
                        .addPath(new BezierCurve(new Point(35, 40), new Point( 29, 40), new Point( 6, 0), new Point(0, 0))).setConstantHeadingInterpolation(Math.toRadians(180)).build();
        follower.followPath(p2pPath);
        p2pPathTimer.resetTimer();
        inP2P = true;
        g1.rumble(100);
        g2.rumble(100);
    }


    double angleDiff(double angle){
        return (Math.abs(angle) < Math.abs((2 * Math.PI - angle))) ? angle : 2 * Math.PI - angle;
    }

    public Gamepad getG1(){
        return this.g1;
    }
    public Gamepad getG2(){
        return this.g2;
    }
    public Telemetry getT(){
        return this.t;
    }

    public boolean getMode(){
        return this.MODE;
    }


}
