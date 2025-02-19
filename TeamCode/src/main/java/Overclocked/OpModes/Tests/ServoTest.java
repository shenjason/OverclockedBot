package Overclocked.OpModes.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;



@Config
@Autonomous(name = "ServoTest", group = "Tests")
public class ServoTest extends LinearOpMode {


    public static float SERVO_POS;
    Servo axonServo; Servo copyServo;

    @Override
    public void runOpMode() throws InterruptedException {

        axonServo = hardwareMap.get(Servo.class, "Servo1");
        axonServo.setDirection(Servo.Direction.REVERSE);
        copyServo = hardwareMap.get(Servo.class, "Servo2");

        waitForStart();

        while (opModeIsActive()){
            axonServo.setPosition(SERVO_POS);
            copyServo.setPosition(SERVO_POS);
        }
    }
}
