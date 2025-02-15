package Overclocked.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "AxonServoTest")
public class AxonServo extends LinearOpMode {

    Servo axonServo;

    @Override
    public void runOpMode() throws InterruptedException {

        axonServo = hardwareMap.get(Servo.class, "axonServo");

        waitForStart();

        while (opModeIsActive()){
            axonServo.setPosition(gamepad1.left_stick_x);
        }
    }
}
