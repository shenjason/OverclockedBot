package Overclocked.OpModes.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;

import Overclocked.Constants.IntakeArmPose;
import Overclocked.Subassemblies.Intake;

@Config
@Autonomous(name="CameraDetectionTest", group = "Tests")
public class CameraDetectionTest extends OpMode {
    VisionPortal visionPortal;
    Intake intake;

    ColorBlobLocatorProcessor myColorBlobLocatorProcessor;
    VisionPortal myVisionPortal;
    List<ColorBlobLocatorProcessor.Blob> myBlobs;
    ColorBlobLocatorProcessor.Blob myBlob;
    RotatedRect myBoxFit;

    @Override
    public void init() {
        ColorBlobLocatorProcessor.Builder myColorBlobLocatorProcessorBuilder;
        myColorBlobLocatorProcessorBuilder = new ColorBlobLocatorProcessor.Builder();

        myColorBlobLocatorProcessorBuilder.setTargetColorRange(ColorRange.BLUE);
        myColorBlobLocatorProcessorBuilder.setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5));
        myColorBlobLocatorProcessorBuilder.setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY);
        myColorBlobLocatorProcessorBuilder.setDrawContours(true);


        VisionPortal.Builder myVisionPortalBuilder;
        intake = new Intake(hardwareMap, null, gamepad1);
        visionPortal = new VisionPortal.Builder().setCamera(hardwareMap.get(CameraName.class, "camera")).enableLiveView(true).build();
    }

    @Override
    public void start() {
        intake.setArmPose(IntakeArmPose.DETECTION);
        super.start();
    }

    @Override
    public void loop() {

    }
}
