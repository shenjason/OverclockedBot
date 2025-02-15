package Overclocked.OpModes.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name="CameraDetectionTest", group = "Tests")
public class CameraDetectionTest extends OpMode {
    VisionPortal visionPortal;

    @Override
    public void init() {
        visionPortal = new VisionPortal.Builder().setCamera(hardwareMap.get(CameraName.class, "camera")).enableLiveView(true).build();
    }

    @Override
    public void start() {

        super.start();
    }

    @Override
    public void loop() {

    }
}
