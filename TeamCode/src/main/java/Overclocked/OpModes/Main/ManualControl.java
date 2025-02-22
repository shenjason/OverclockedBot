package Overclocked.OpModes.Main;


import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import Overclocked.Constants.IntakeArmPose;
import Overclocked.Constants.IntakeSlidePose;
import Overclocked.Constants.OuttakeArmPose;
import Overclocked.Constants.OuttakeSlidePose;
import Overclocked.Subassemblies.Intake;
import Overclocked.Subassemblies.Outtake;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@TeleOp(name = "ManualControl", group = "Main")
public class ManualControl extends LinearOpMode {

    public boolean Mode;

    public Outtake outtake;
    public Intake intake;
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);
    DcMotor lf; DcMotor rf; DcMotor lr; DcMotor rr;

    boolean isBpressed; boolean isXpressed; boolean isApressed; boolean isB2pressed;
    boolean isA2pressed; boolean isX2pressed; boolean isYpressed; boolean switching;

    Timer timer;


    @Override
    public void runOpMode() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        Mode = true;

        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap, outtake);



        waitForStart();

        timer = new Timer();

        outtake.setSlidePose(OuttakeSlidePose.INITIAL);
        outtake.setOuttakeArmPose(OuttakeArmPose.TRANSFER);
        intake.setArmPose(IntakeArmPose.TRANSFER);
        intake.setSlidePose(IntakeSlidePose.FREE);




        follower.startTeleopDrive();

        lf = hardwareMap.get(DcMotor.class, "leftFront");
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf = hardwareMap.get(DcMotor.class, "rightFront");
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr = hardwareMap.get(DcMotor.class, "leftRear");
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr = hardwareMap.get(DcMotor.class, "rightRear");
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        while (opModeIsActive()) {
            float speed = 1;
            if (gamepad1.left_bumper || gamepad1.right_bumper){
                speed = 0.5f;
            }

            follower.setTeleOpMovementVectors(Math.pow(Math.abs(gamepad1.left_stick_y), 3) * -Math.signum(gamepad1.left_stick_y) * speed, Math.pow(Math.abs(gamepad1.left_stick_x), 3) * -Math.signum(gamepad1.left_stick_x) * speed, Math.pow(Math.abs(gamepad1.right_stick_x), 3) * -Math.signum(gamepad1.right_stick_x) * speed * 0.7, true);
            follower.update();



            if (!Mode) SpecimenOuttakeControls();
            else SampleOuttakeControls();

            if (!gamepad2.b) isBpressed = false;
            if (gamepad2.b && !isBpressed) {
                outtake.switchClawState();
                isBpressed = true;
            }

            if (!gamepad2.x) isX2pressed = false;
            if (gamepad2.x && !isX2pressed) {
                intake.switchClawState();
                isX2pressed = true;
            }



            if (!gamepad2.dpad_down) isXpressed = false;
            if (gamepad2.dpad_down && !isXpressed && (outtake.isSlidePose(OuttakeSlidePose.INITIAL) || outtake.isSlidePose(OuttakeSlidePose.SWITCH)) && intake.isSlideAtInitial()) {
                Mode = !Mode;
                if (Mode){
                    outtake.setOuttakeArmPose(OuttakeArmPose.TRANSFER);
                    intake.setSlidePose(IntakeSlidePose.FREE);
                }
                else {
                    outtake.setOuttakeArmPose(OuttakeArmPose.SPECIMEN_PICKUP);
                    outtake.setSlidePose(OuttakeSlidePose.INITIAL);
                    intake.setArmPose(IntakeArmPose.INACTIVE);
                }

                gamepad2.rumble(100);

                isXpressed = true;
            }


            if (Mode) {
                SampleIntakeControls();
            }
            intake.update(Mode);
            outtake.update();


            /* Telemetry Outputs of our Follower */
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

            /* Update Telemetry to the Driver Hub */
            telemetry.update();
        }

    }

    void SpecimenOuttakeControls(){
        if (!gamepad2.right_bumper) isApressed = false;
        if (gamepad2.right_bumper && outtake.canChangeSlidePoseTimed() && !isApressed){
            if (outtake.isSlidePose(OuttakeSlidePose.INITIAL)){
                outtake.setSlidePose(OuttakeSlidePose.SPECIMEN_SCORE);
                outtake.setOuttakeArmPose(OuttakeArmPose.SPECIMEN_SCORE);
            }else{
                outtake.pullDownSpecimen();
            }
            isApressed = true;
        }
    }

    void SampleIntakeControls(){
        intake.slideControl(-gamepad2.left_stick_y);

        if (!gamepad2.a) isA2pressed = false;
        if (gamepad2.a && outtake.canChangeSlidePoseTimed() && !isA2pressed){
            if (intake.isArmPose(IntakeArmPose.DETECTION)) intake.transfer();
            isA2pressed = true;
        }

        if (!gamepad2.left_bumper) isYpressed = false;
        if (gamepad2.left_bumper && !isYpressed){
            intake.pickup();
            isYpressed = true;
        }

        if (gamepad2.right_stick_x > 0.5){
            intake.setOverride_auto_rotation(2);
        } else if (gamepad2.right_stick_x < -0.5){
            intake.setOverride_auto_rotation(1);
        }else{
            intake.setOverride_auto_rotation(0);
        }


    }

    void SampleOuttakeControls(){
        if (!gamepad2.right_bumper) isApressed = false;
        if (gamepad2.right_bumper && outtake.canChangeSlidePoseTimed() && !isApressed){
            if (outtake.isSlidePose(OuttakeSlidePose.INITIAL)){
                outtake.setSlidePose(OuttakeSlidePose.SAMPLE_SCORE);
                outtake.setOuttakeArmPose(OuttakeArmPose.SAMPLE_SCORE);
            }else{
                outtake.setSlidePose(OuttakeSlidePose.INITIAL);
                outtake.setOuttakeArmPose(OuttakeArmPose.TRANSFER);
            }
            isApressed = true;
        }
    }
}

