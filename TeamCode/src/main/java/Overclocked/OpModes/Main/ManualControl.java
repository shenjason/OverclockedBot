package Overclocked.OpModes.Main;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import Overclocked.Constants.OuttakeArmPose;
import Overclocked.Constants.OuttakeSlidePose;
import Overclocked.Subassemblies.Outtake;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@TeleOp(name = "ManualControl", group = "Main")
public class ManualControl extends LinearOpMode {

    public boolean Mode = false;

    public Outtake outtake;
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);
    DcMotor lf; DcMotor rf; DcMotor lr; DcMotor rr;

    boolean isBpressed; boolean isXpressed;


    @Override
    public void runOpMode() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        //Prevent robot from drifting

        outtake = new Outtake(hardwareMap);

        waitForStart();
        outtake.setOuttakeArmPose(OuttakeArmPose.SPECIMEN_PICKUP);

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

            follower.setTeleOpMovementVectors(Math.pow(Math.abs(gamepad1.left_stick_y), 3) * -Math.signum(gamepad1.left_stick_y), Math.pow(Math.abs(gamepad1.left_stick_x), 3) * -Math.signum(gamepad1.left_stick_x), Math.pow(Math.abs(gamepad1.right_stick_x), 3) * -Math.signum(gamepad1.right_stick_x), true);
            follower.update();

            if (!Mode) SpecimenOuttakeControls();

            if (!gamepad1.b) isBpressed = false;
            if (gamepad1.b && !isBpressed) {
                outtake.switchClawState();
                isBpressed = true;
            }

            if (!gamepad1.x) isXpressed = false;
            if (gamepad1.x && !isXpressed) {
                Mode = !Mode;
                isXpressed = true;
            }


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
        if (gamepad1.a && outtake.canChangeSlidePose()){
            if (outtake.isSlidePose(OuttakeSlidePose.INITIAL)){
                outtake.setSlidePose(OuttakeSlidePose.SPECIMEN_SCORE);
                outtake.setOuttakeArmPose(OuttakeArmPose.SPECIMEN_SCORE);
            }else{
                outtake.setSlidePose(OuttakeSlidePose.INITIAL);
                outtake.setOuttakeArmPose(OuttakeArmPose.SPECIMEN_PICKUP);
            }
        }
    }
}

