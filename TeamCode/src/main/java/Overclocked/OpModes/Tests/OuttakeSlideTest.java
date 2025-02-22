package Overclocked.OpModes.Tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;




/*

 */
@TeleOp(name = "OuttakeSlideTest(Outdated)", group = "Tests")
public class OuttakeSlideTest extends LinearOpMode {
    DcMotor upperOuttakeSlide;
    DcMotor bottomOuttakeSlide;
    TouchSensor OuttakeSwitch;

    int upperSlidePos = 0; int bottomSlidePos = 0;


    @Override
    public void runOpMode() {
        upperOuttakeSlide = hardwareMap.get(DcMotor.class, "UpperOutakeSlide");
        bottomOuttakeSlide = hardwareMap.get(DcMotor.class, "BottomOutakeSlide");
        OuttakeSwitch = hardwareMap.get(TouchSensor.class, "mag");


        upperOuttakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomOuttakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        bottomOuttakeSlide.setDirection(DcMotorSimple.Direction.REVERSE);


        upperSlidePos = 0;
        bottomSlidePos = 0;
        upperOuttakeSlide.setTargetPosition(upperSlidePos);
        bottomOuttakeSlide.setTargetPosition(bottomSlidePos);
        upperOuttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomOuttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        upperOuttakeSlide.setPower(1);
        bottomOuttakeSlide.setPower(1);

        waitForStart();

        while (opModeIsActive()){
            if (OuttakeSwitch.isPressed()){
                telemetry.addLine("reset");
                upperOuttakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bottomOuttakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                upperOuttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bottomOuttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            telemetry.update();
            if (gamepad1.a){
                upperOuttakeSlide.setPower(1);
                bottomOuttakeSlide.setPower(1);
                upperSlidePos = 1300;
                bottomSlidePos = 1300;
            } if (gamepad1.b){
                upperOuttakeSlide.setPower(0.75);
                bottomOuttakeSlide.setPower(0.75);
                upperSlidePos = 0;
                bottomSlidePos = 0;
            }
            upperOuttakeSlide.setTargetPosition(upperSlidePos);
            bottomOuttakeSlide.setTargetPosition(bottomSlidePos);
        }
    }
}
