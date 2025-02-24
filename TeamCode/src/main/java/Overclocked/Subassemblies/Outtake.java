package Overclocked.Subassemblies;

import android.transition.Slide;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import Overclocked.Constants.OuttakeArmPose;
import Overclocked.Constants.OuttakeConstants;
import Overclocked.Constants.OuttakeSlidePose;

public class Outtake {

    public DcMotor upperSlide;
    public DcMotor bottomSlide;

    public TouchSensor magSwitch;

    public Servo armLeft; public Servo armRight; Servo wristLeft; Servo wristRight; Servo wristMid; Servo claw;

    int armPose; int slidePose; boolean clawState; boolean isInPullDown;
    boolean isReseted;

    Timer sincePoseSwitch; Timer pulldownTimer;
    
    public Outtake(HardwareMap hardwareMap){
        sincePoseSwitch = new Timer();
        pulldownTimer = new Timer();
        hardwareInit(hardwareMap);
    }


    //Hardware Inits
    public void hardwareInit(HardwareMap hardwareMap){
        slideHardwareInit(hardwareMap);
        armHardwareInit(hardwareMap);
        sensorHardwareInit(hardwareMap);
    }

    void armHardwareInit(HardwareMap hardwareMap){
        armLeft = hardwareMap.get(Servo.class, "outtakeArmLeft");
        armLeft.setDirection(Servo.Direction.REVERSE);
        armRight = hardwareMap.get(Servo.class, "outtakeArmRight");

        wristLeft = hardwareMap.get(Servo.class, "outtakeWristLeft");
        wristRight = hardwareMap.get(Servo.class, "outtakeWristRight");
        wristRight.setDirection(Servo.Direction.REVERSE);

        wristMid = hardwareMap.get(Servo.class, "outtakeWristMid");
        wristMid.setDirection(Servo.Direction.REVERSE);

        claw = hardwareMap.get(Servo.class, "outtakeClaw");
        claw.setDirection(Servo.Direction.REVERSE);
    }

    void slideHardwareInit(HardwareMap hardwareMap){
        upperSlide = hardwareMap.get(DcMotor.class, "upperOuttakeSlide");
        bottomSlide = hardwareMap.get(DcMotor.class, "bottomOuttakeSlide");

        upperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        upperSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        slideEncoderReset();
    }

    void sensorHardwareInit(HardwareMap hardwareMap){
        magSwitch = hardwareMap.get(TouchSensor.class, "mag");
    }


    public void setSlidePose(int pose){
        slidePose = pose;


        switch (pose){
            case (OuttakeSlidePose.INITIAL):
                upperSlide.setTargetPosition(-15);
                bottomSlide.setTargetPosition(-15);
                break;
            case (OuttakeSlidePose.SAMPLE_SCORE):
                upperSlide.setTargetPosition(1300);
                bottomSlide.setTargetPosition(1300);
                break;
            case (OuttakeSlidePose.SPECIMEN_SCORE):
                upperSlide.setTargetPosition(OuttakeConstants.specimen_hang_slide_pos);
                bottomSlide.setTargetPosition(OuttakeConstants.specimen_hang_slide_pos);
                break;
            case (OuttakeSlidePose.SPECIMEN_SCORE_AUTO):
                upperSlide.setTargetPosition(600);
                bottomSlide.setTargetPosition(600);
                break;
            case (OuttakeSlidePose.SWITCH):
                upperSlide.setTargetPosition(200);
                bottomSlide.setTargetPosition(200);
                break;
        }

        if (bottomSlide.getCurrentPosition() <= bottomSlide.getTargetPosition()){
            upperSlide.setPower(OuttakeConstants.slide_extension_power);
            bottomSlide.setPower(OuttakeConstants.slide_extension_power);
        }else{
            upperSlide.setPower(OuttakeConstants.slide_retraction_power);
            bottomSlide.setPower(OuttakeConstants.slide_retraction_power);
        }

        upperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setOuttakeArmPose(int pose){
        armPose = pose;
        sincePoseSwitch.resetTimer();

        switch (pose){
            case OuttakeArmPose.TRANSFER:
                armLeft.setPosition(0.62);
                armRight.setPosition(0.62);

                wristRight.setPosition(0);
                wristLeft.setPosition(0);

                wristMid.setPosition(0.210);

                setClawState(OuttakeArmPose.CLAW_OPEN);
                break;
            case OuttakeArmPose.SPECIMEN_PICKUP:
                armLeft.setPosition(0.8);
                armRight.setPosition(0.8);

                wristRight.setPosition(0.5);
                wristLeft.setPosition(0.5);

                wristMid.setPosition(0.210);

                setClawState(OuttakeArmPose.CLAW_OPEN);
                break;
            case OuttakeArmPose.SPECIMEN_SCORE:
                armLeft.setPosition(0.1);
                armRight.setPosition(0.1);

                wristRight.setPosition(0.19);
                wristLeft.setPosition(0.19);

                wristMid.setPosition(0.323);

//                setClawState(OuttakeArmPose.CLAW_CLOSE);
                break;

            case OuttakeArmPose.SAMPLE_SCORE:
                armLeft.setPosition(0.4);
                armRight.setPosition(0.4);

                wristRight.setPosition(0.6);
                wristLeft.setPosition(0.6);

                wristMid.setPosition(0.323);

                break;
        }
    }

    public void secureWrist(){
        wristRight.setPosition(0.5);
        wristLeft.setPosition(0.5);
    }

    public void deactivateSlides(){
        upperSlide.setPower(0);
        bottomSlide.setPower(0);
    }

    public void setClawState(boolean state){
        clawState = state;
        if (state){
            claw.setPosition(0.1);
            return;
        }
        claw.setPosition(0.3);
    }

    public void switchClawState(){
        setClawState(!clawState);
    }


    public void slideEncoderReset(){
        upperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        upperSlide.setTargetPosition(0);
        bottomSlide.setTargetPosition(0);
    }

    public boolean canChangeSlidePose(){
        return Math.abs(bottomSlide.getCurrentPosition() - bottomSlide.getTargetPosition()) < OuttakeConstants.slide_pose_switch_tolerance;
    }


    public boolean canChangeSlidePoseTimed(){
        return Math.abs(bottomSlide.getCurrentPosition() - bottomSlide.getTargetPosition()) < OuttakeConstants.slide_pose_switch_tolerance || sincePoseSwitch.getElapsedTimeSeconds() > 3;
    }

    public boolean isSlidePose(int pose){
        return (slidePose == pose);
    }
    public boolean isArmPose(int pose){
        return (armPose == pose);
    }

    public boolean isClawState(boolean state){
        return (clawState == state);
    }

    public void pullDownSpecimen(){
        setSlidePose(OuttakeSlidePose.INITIAL);
        secureWrist();

        isInPullDown = true;

        pulldownTimer.resetTimer();
    }


    public void update(){
        if (!magSwitch.isPressed()) isReseted = false;
        if (magSwitch.isPressed() && !isReseted){
            slideEncoderReset();
            isReseted = true;
        }
        if (canChangeSlidePose() && isSlidePose(OuttakeSlidePose.INITIAL)){
            deactivateSlides();
        }

        if (isInPullDown && pulldownTimer.getElapsedTimeSeconds() > 0.2){
           isInPullDown = false;
           setOuttakeArmPose(OuttakeArmPose.SPECIMEN_PICKUP);
        }
    }





}



