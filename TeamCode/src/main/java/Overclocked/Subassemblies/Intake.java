package Overclocked.Subassemblies;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import Overclocked.Constants.IntakeArmPose;
import Overclocked.Constants.IntakeConstants;
import Overclocked.Constants.IntakeSlidePose;


public class Intake {

    public DcMotor intakeSlide;

    public TouchSensor magSwitch;
    public Servo armLeft; public Servo armRight; Servo wristLeft; Servo wristRight; Servo wristMid; Servo claw;

    int armPose; int slidePose; boolean clawState; boolean isReseted; boolean isSlideFree;

    Timer pickupTimer; Timer transferTimer; Timer autoAngleTimer; Timer detectionTimer; Timer pickupModeTimer;

    boolean canSwitchToDetection; boolean isPickup;
    public boolean allowedToSwitchToDetection;


    boolean Detection;

    int override_auto_rotation; int auto_slide_position;


    public Intake(HardwareMap hardwareMap){
        hardwareInit(hardwareMap);
        pickupTimer = new Timer();
        transferTimer = new Timer();
        autoAngleTimer = new Timer();
        detectionTimer = new Timer();
        pickupModeTimer = new Timer();

        allowedToSwitchToDetection = true;
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
        intakeSlide.setTargetPosition(0);
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
            claw.setPosition(0.25);//claw open 0.2
            return;
        }
        claw.setPosition(0.45);//claw close
    }

    public void switchClawState(){
        setClawState(!clawState);
    }

    public boolean slideControl(double power){
        if (!isSlideFree) return false;
        double corrected_power = power;
        if (intakeSlide.getCurrentPosition() > IntakeConstants.max_extension_pos) {
            corrected_power = -Math.abs(power);
            intakeSlide.setPower(corrected_power);
            return false;
        }
        intakeSlide.setPower(corrected_power);
        return true;
    }

    public void setWristMidAngle(double angle){
        if (isPickup) return;
        double pos = IntakeConstants.deg0_wrist_pos + (angle / 180) * (IntakeConstants.deg180_wrist_pos - IntakeConstants.deg0_wrist_pos);
        wristMid.setPosition(pos);
    }


    public void setSlidePose(int pose){
        slidePose = pose;
        isSlideFree = (slidePose == IntakeSlidePose.FREE || slidePose == IntakeSlidePose.INITIAL || slidePose == IntakeSlidePose.AUTO_PICKUP);
        switch (pose){
            case (IntakeSlidePose.AUTO_PICKUP):
            case (IntakeSlidePose.INITIAL):
            case (IntakeSlidePose.FREE):
                intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                intakeSlide.setPower(0);
                break;
            case (IntakeSlidePose.TRANSFER):
                intakeSlide.setTargetPosition(20);
                break;
            case (IntakeSlidePose.AUTO_POSITION):
                intakeSlide.setTargetPosition(auto_slide_position);
                break;
        }
        if (!isSlideFree){
            if (intakeSlide.getCurrentPosition() < intakeSlide.getTargetPosition()){
                intakeSlide.setPower(IntakeConstants.slide_extension_power);
            }else{
                intakeSlide.setPower(IntakeConstants.slide_retraction_power);
            }

            intakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }


    public void setArmPose(int pose){
        armPose = pose;
        Detection = (pose == IntakeArmPose.DETECTION);

        switch (pose){
            case (IntakeArmPose.INACTIVE):
                armRight.setPosition(0.08);
                armLeft.setPosition(0.08);

                wristLeft.setPosition(0.9);
                wristRight.setPosition(0.9);

                wristMid.setPosition(0.484);

                setClawState(IntakeArmPose.CLAW_OPEN);
                break;
            case (IntakeArmPose.TRANSFER):
                armRight.setPosition(0.55);
                armLeft.setPosition(0.55);

                wristLeft.setPosition(0.2);
                wristRight.setPosition(0.2);

                wristMid.setPosition(0.367);

                break;
            case (IntakeArmPose.DETECTION):
                armRight.setPosition(0.53);
                armLeft.setPosition(0.53);

                wristLeft.setPosition(1);
                wristRight.setPosition(1);

//                wristMid.setPosition(IntakeConstants.deg0_wrist_pos);

                break;
            case (IntakeArmPose.SAMPLE_PICKUP):

                armRight.setPosition(0.69);
                armLeft.setPosition(0.69);

                wristLeft.setPosition(0.93);
                wristRight.setPosition(0.93);

                setClawState(IntakeArmPose.CLAW_OPEN);
                break;
            case (IntakeArmPose.SWEEP):
                armRight.setPosition(0.72);
                armLeft.setPosition(0.72);

                wristLeft.setPosition(0.8);
                wristRight.setPosition(0.8);
                break;
        }
    }

    public void setOverride_auto_rotation(int override){
        override_auto_rotation = override;
    }

    public void pickup(){
        pickupTimer.resetTimer();
        isPickup = true;
        setArmPose(IntakeArmPose.SAMPLE_PICKUP);
    }

    public boolean isSlideAtInitial(){
        return Math.abs(intakeSlide.getCurrentPosition()) < 25;
    }

    public void autoSetSlidePos(int pos){
        allowedToSwitchToDetection = false;
        auto_slide_position = pos;
        setSlidePose(IntakeSlidePose.AUTO_POSITION);
    }


    public void update(Telemetry telemetry){
        if (!magSwitch.isPressed()){
            isReseted = false;
            if (isSlidePose(IntakeSlidePose.INITIAL)){
                intakeSlide.setPower(-1);
            }
        }else{
            if (isSlidePose(IntakeSlidePose.INITIAL)){
                intakeSlide.setPower(0);
            }
        }

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


        if (intakeSlide.getCurrentPosition() > IntakeConstants.detection_switch_tick_pos && canSwitchToDetection && allowedToSwitchToDetection){
            setArmPose(IntakeArmPose.DETECTION);
            canSwitchToDetection = false;
        }

        if (intakeSlide.getCurrentPosition() < 50){
            canSwitchToDetection = true;
        }


        if (Detection){
            if (override_auto_rotation != 0){
                switch (override_auto_rotation){
                    case 1:
                        setWristMidAngle(0);
                        break;
                    case 2:
                        setWristMidAngle(45);
                        break;
                    case 3:
                        setWristMidAngle(-45);
                        break;
                    case 4:
                        setWristMidAngle(90);
                }
            }
        }

    }
}
