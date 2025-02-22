package Overclocked.OpModes.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;



@Config
@Autonomous(name = "ServoTest", group = "Tests")
public class ServoTest extends LinearOpMode {

    public static String REVERSED_SERVO_NAME = "";
    public static String FORWARD_SERVO_NAME = "";
    public static float SERVO_POS;
    Servo axonServo; Servo copyServo;

    @Override
    public void runOpMode() {

        axonServo = hardwareMap.get(Servo.class, REVERSED_SERVO_NAME);
        axonServo.setDirection(Servo.Direction.REVERSE);
        copyServo = hardwareMap.get(Servo.class, FORWARD_SERVO_NAME);

        waitForStart();

        while (opModeIsActive()){
            axonServo.setPosition(SERVO_POS);
            copyServo.setPosition(SERVO_POS);
        }
    }
}
