package Overclocked.OpModes.Tests;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.SortOrder;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import Overclocked.Constants.IntakeArmPose;
import Overclocked.Constants.IntakeConstants;
import Overclocked.Subassemblies.Intake;


@Config
@Autonomous(name = "CameraDetectionTest", group = "Tests")
public class CameraDetectionTest extends LinearOpMode {
    public static int MIN_AREA = 50;

    public static int RES_WIDTH = 320;

    public static int RES_HEIGHT = 160;

    boolean isApressed;

    @Override
    public void runOpMode() {
        ColorBlobLocatorProcessor.Builder myColorBlobLocatorProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;
        ColorBlobLocatorProcessor myColorBlobLocatorProcessor;
        VisionPortal myVisionPortal;
        List<ColorBlobLocatorProcessor.Blob> myBlobs;

        Intake intake;

        myColorBlobLocatorProcessorBuilder = new ColorBlobLocatorProcessor.Builder();
        myColorBlobLocatorProcessorBuilder.setTargetColorRange(ColorRange.BLUE);
        myColorBlobLocatorProcessorBuilder.setRoi(ImageRegion.entireFrame());
        myColorBlobLocatorProcessorBuilder.setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY);
        myColorBlobLocatorProcessorBuilder.setDrawContours(true);
        myColorBlobLocatorProcessorBuilder.setBlurSize(5);
        myColorBlobLocatorProcessor = myColorBlobLocatorProcessorBuilder.build();
        myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortalBuilder.addProcessor(myColorBlobLocatorProcessor);
        myVisionPortalBuilder.setCameraResolution(new Size(160, 120));
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "camera"));
        myVisionPortal = myVisionPortalBuilder.build();
        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        intake = new Intake(hardwareMap);

        intake.setArmPose(IntakeArmPose.DETECTION);


        while (opModeIsActive() || opModeInInit()) {
            telemetry.addData("preview on/off", "... Camera Stream");

            myBlobs = myColorBlobLocatorProcessor.getBlobs();
            ColorBlobLocatorProcessor.Util.filterByArea(MIN_AREA, 2000000, myBlobs);
            ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, myBlobs);
            telemetry.addData("Blobs", myBlobs.size());

            if (!myBlobs.isEmpty()){
                ColorBlobLocatorProcessor.Blob Blob = myBlobs.get(0);

                telemetry.addData("x", Blob.getBoxFit().center.y);
                telemetry.addData("y", Blob.getBoxFit().center.x);
                telemetry.addData("width", Blob.getBoxFit().size.width);
                telemetry.addData("height", Blob.getBoxFit().size.height);
                telemetry.addData("area", Blob.getContourArea());
                telemetry.addData("angle", filterAngle(Blob.getBoxFit()));



                intake.setWristMidAngle(filterAngle(Blob.getBoxFit()) - 90);

            }

            if (!gamepad1.a) isApressed = false;
            if (gamepad1.a && !isApressed){
                intake.pickup();
                isApressed = true;
            }

            intake.update(telemetry);

            telemetry.update();
        }
    }


    double filterAngle(RotatedRect box) {
        Point[] p = new Point[4];
        box.points(p);
        double angle;
        if (distanceSq(p[0], p[1]) > distanceSq(p[1], p[2])){
            angle =  Math.toDegrees(Math.atan2(p[0].x - p[1].x, p[0].y - p[1].y));
        }else{
            angle =  Math.toDegrees(Math.atan2(p[1].x - p[2].x, p[1].y - p[2].y));
        }
//        if (angle < 5) angle += 180;
        return angle;
    }

    double distanceSq(Point a, Point b){
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return dx*dx + dy*dy;
    }
}