package Overclocked.OpModes.Tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "OuttakeSlideTest")
public class OuttakeSlideTest extends LinearOpMode {
    DcMotor upperOuttakeSlide;
    DcMotor bottomOuttakeSlide;


    @Override
    public void runOpMode() throws InterruptedException {
        upperOuttakeSlide = hardwareMap.get(DcMotor.class, "upperOutakeSlide");
        bottomOuttakeSlide = hardwareMap.get(DcMotor.class, "bottomOutakeSlide");

        upperOuttakeSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomOuttakeSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bottomOuttakeSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.a){
                upperOuttakeSlide.setPower(1);
                bottomOuttakeSlide.setPower(1);
            }else if (gamepad1.b){
                upperOuttakeSlide.setPower(-1);
                bottomOuttakeSlide.setPower(-1);
            }else{
                upperOuttakeSlide.setPower(0);
                bottomOuttakeSlide.setPower(0);
            }
        }
    }
}
