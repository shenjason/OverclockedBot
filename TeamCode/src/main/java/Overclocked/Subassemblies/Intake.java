package Overclocked.Subassemblies;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


import Overclocked.Constants.IntakeArmPose;
import Overclocked.Constants.IntakeConstants;
import Overclocked.Constants.IntakeSlidePose;
import Overclocked.Constants.OuttakeArmPose;


public class Intake {

    public DcMotor intakeSlide;

    public TouchSensor magSwitch;
    public Servo armLeft; public Servo armRight; Servo wristLeft; Servo wristRight; Servo wristMid; Servo claw;

    int armPose; int slidePose; boolean clawState; boolean isReseted; boolean isSlideFree;

    Timer pickupTimer; Timer transferTimer;

    boolean canSwitchToDetection; boolean isPickup; boolean inTransfer1; boolean inTransfer2;

    public Outtake linkedOuttake;

    boolean Detection;

    int override_auto_rotation;

    Gamepad gamepad;

    public Intake(HardwareMap hardwareMap, Outtake outtake, Gamepad controller){
        hardwareInit(hardwareMap);
        pickupTimer = new Timer();
        transferTimer = new Timer();

        gamepad = controller;

        linkedOuttake = outtake;
    }

    public void hardwareInit(HardwareMap hardwareMap){
        slideHardwareInit(hardwareMap);
        armHardwareInit(hardwareMap);
        sensorHardwareInit(hardwareMap);
    }

    void slideHardwareInit(HardwareMap hardwareMap){
        intakeSlide = hardwareMap.get(DcMotor.class, "intakeSlide");

        intakeSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideEncoderReset();
    }


    void armHardwareInit(HardwareMap hardwareMap){
        armLeft = hardwareMap.get(Servo.class, "intakeArmLeft");
        armRight = hardwareMap.get(Servo.class, "intakeArmRight");
        armRight.setDirection(Servo.Direction.REVERSE);

        wristLeft = hardwareMap.get(Servo.class, "intakeWristLeft");
        wristRight = hardwareMap.get(Servo.class, "intakeWristRight");
        wristRight.setDirection(Servo.Direction.REVERSE);

        wristMid = hardwareMap.get(Servo.class, "intakeWristMid");

        claw = hardwareMap.get(Servo.class, "intakeClaw");
        claw.setDirection(Servo.Direction.REVERSE);
    }

    void sensorHardwareInit(HardwareMap hardwareMap){
        magSwitch = hardwareMap.get(TouchSensor.class, "mag2");
    }


    public void slideEncoderReset(){
        intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (isSlideFree) intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void slideControl(double power){
        if (!isSlideFree) return;
        if (intakeSlide.getCurrentPosition() > IntakeConstants.max_extension_pos) {
            rumbleGamepad();
            intakeSlide.setPower(0);
            return;
        }
        intakeSlide.setPower(power);
    }

    public void transfer(){
        inTransfer1 = true;
        setArmPose(IntakeArmPose.TRANSFER);
        linkedOuttake.setClawState(OuttakeArmPose.CLAW_OPEN);
        transferTimer.resetTimer();
    }

    public void setWristMidAngle(double angle){
       double pos = IntakeConstants.deg0_wrist_pos + (angle / 180) * (IntakeConstants.deg180_wrist_pos - IntakeConstants.deg0_wrist_pos);
       wristMid.setPosition(pos);
    }


    public void setSlidePose(int pose){
        slidePose = pose;
        isSlideFree = (slidePose == IntakeSlidePose.FREE);
        switch (pose){
            case (IntakeSlidePose.INITIAL):
                intakeSlide.setTargetPosition(-25);
                break;
            case (IntakeSlidePose.AUTO_PICKUP):
                intakeSlide.setTargetPosition(400);
                break;
            case (IntakeSlidePose.FREE):
                intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                intakeSlide.setPower(0);
                break;
        }
        if (!isSlideFree){
            intakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (intakeSlide.getCurrentPosition() < intakeSlide.getTargetPosition()){
                intakeSlide.setPower(IntakeConstants.slide_extension_power);
            }else{
                intakeSlide.setPower(IntakeConstants.slide_retraction_power);
            }
        }
    }


    public void setArmPose(int pose){
        armPose = pose;
        Detection = (pose == IntakeArmPose.DETECTION);

        switch (pose){
            case (IntakeArmPose.INACTIVE):
                armRight.setPosition(0.1);
                armLeft.setPosition(0.1);

                wristLeft.setPosition(0.83);
                wristRight.setPosition(0.83);

                wristMid.setPosition(0.484);

                setClawState(IntakeArmPose.CLAW_OPEN);
                break;
            case (IntakeArmPose.TRANSFER):
                armRight.setPosition(0.1);
                armLeft.setPosition(0.1);

                wristLeft.setPosition(1);
                wristRight.setPosition(1);

                wristMid.setPosition(0.367);

//                setClawState(IntakeArmPose.CLAW_CLOSE);
                break;
            case (IntakeArmPose.DETECTION):
                armRight.setPosition(0.7);
                armLeft.setPosition(0.7);

                wristLeft.setPosition(1);
                wristRight.setPosition(1);

                wristMid.setPosition(0.367);

//                setClawState(IntakeArmPose.CLAW_OPEN);
                break;
            case (IntakeArmPose.SAMPLE_PICKUP):
                armRight.setPosition(0.85);
                armLeft.setPosition(0.85);

                wristLeft.setPosition(0.92);
                wristRight.setPosition(0.92);

//                wristMid.setPosition(0.367);

                setClawState(IntakeArmPose.CLAW_OPEN);
                break;

        }
    }

    public void setOverride_auto_rotation(int override){
        override_auto_rotation = override;
    }


    public void pickup(){
        if (inTransfer1 || inTransfer2) return;
        pickupTimer.resetTimer();
        isPickup = true;
        setArmPose(IntakeArmPose.SAMPLE_PICKUP);
    }

    public boolean isSlideAtInitial(){
        return Math.abs(intakeSlide.getCurrentPosition()) < 25;
    }


    public void rumbleGamepad(){
        if (gamepad == null) return;
        gamepad.rumble(100);
    }


    public void update(boolean Mode){
        if (!magSwitch.isPressed()) isReseted = false;
        if (magSwitch.isPressed() && !isReseted){
            slideEncoderReset();
            isReseted = true;
        }

        if (pickupTimer.getElapsedTimeSeconds() > IntakeConstants.time_after_pickupstate_close_claw && isPickup){
            setClawState(IntakeArmPose.CLAW_CLOSE);
        }
        if (pickupTimer.getElapsedTimeSeconds() > IntakeConstants.time_after_pickupstate_retract && isPickup){
            isPickup = false;
            setArmPose(IntakeArmPose.DETECTION);
        }


        if (Mode && intakeSlide.getCurrentPosition() > IntakeConstants.detection_switch_tick_pos && canSwitchToDetection){
            setArmPose(IntakeArmPose.DETECTION);
            canSwitchToDetection = false;
        }

        if (intakeSlide.getCurrentPosition() < 50){
            canSwitchToDetection = true;
        }


        if (inTransfer1 && transferTimer.getElapsedTimeSeconds() > 0.4){
            setSlidePose(IntakeSlidePose.INITIAL);
            if (isSlideAtInitial()){
                inTransfer2 = true;
                inTransfer1 = false;
                transferTimer.resetTimer();
            }
        }

        if (inTransfer2){
            linkedOuttake.setClawState(OuttakeArmPose.CLAW_CLOSE);
            if (transferTimer.getElapsedTimeSeconds() > 0.25){
                setClawState(IntakeArmPose.CLAW_OPEN);
                setSlidePose(IntakeSlidePose.FREE);
                inTransfer2 = false;
                rumbleGamepad();
            }
        }

        if (Detection){
            if (override_auto_rotation == 0){
                setWristMidAngle(0); // REPLACED
            }else if (override_auto_rotation == 2){
                setWristMidAngle(90);
            } else {
                setWristMidAngle(0);
            }
        }

    }


}
