package Overclocked.Subassemblies;

import android.util.Size;

import com.fasterxml.jackson.databind.ObjectReader;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.List;

import Overclocked.Constants.IntakeConstants;

public class CameraDetection {
    ColorBlobLocatorProcessor myColorBlobLocatorProcessor;
    VisionPortal myVisionPortal;
    List<ColorBlobLocatorProcessor.Blob> myBlobs;

    boolean debug;
    Telemetry t;

    public boolean hasDetection = false;

    public double angle, ErrorX, ErrorY = 0;


    public CameraDetection(HardwareMap hardwareMap, Telemetry telemetry, boolean debug, boolean color) {
        ColorBlobLocatorProcessor.Builder myColorBlobLocatorProcessorBuilder = new ColorBlobLocatorProcessor.Builder();
        ColorRange colorRange = ColorRange.BLUE;
        if (color == IntakeConstants.SIDE_RED) colorRange = ColorRange.RED;

        myColorBlobLocatorProcessorBuilder.setTargetColorRange(colorRange);
        myColorBlobLocatorProcessorBuilder.setRoi(ImageRegion.entireFrame());

        myColorBlobLocatorProcessorBuilder.setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY);


        myColorBlobLocatorProcessorBuilder.setBlurSize(5);
        myColorBlobLocatorProcessor = myColorBlobLocatorProcessorBuilder.build();
        VisionPortal.Builder myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortalBuilder.addProcessor(myColorBlobLocatorProcessor);
        myVisionPortalBuilder.setCameraResolution(new Size(160, 120));
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "camera"));
        myVisionPortal = myVisionPortalBuilder.build();
//        if (debug){
//            telemetry.setMsTransmissionInterval(50);
//            telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
//        }

        this.debug = debug;
        this.t = telemetry;
    }


    public boolean update() {
        myBlobs = myColorBlobLocatorProcessor.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(IntakeConstants.MIN_AREA, IntakeConstants.MAX_AREA, myBlobs);
        ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, myBlobs);
        angle = 1000;

        if (debug) t.addLine("CAMERA");
        hasDetection = false;
        if (!myBlobs.isEmpty()) {

            hasDetection = true;
            ColorBlobLocatorProcessor.Blob Blob = myBlobs.get(0);

            if (Blob.getContourArea() > 800) {
                angle = filterAngle(Blob.getBoxFit()) -  90;
            }

            ErrorX = IntakeConstants.PICKUP_X - Blob.getBoxFit().center.x;
            ErrorY = IntakeConstants.PICKUP_Y - Blob.getBoxFit().center.y;

            if (debug) t.addData("angle", angle);
            if (debug) t.addData("area", Blob.getContourArea());
            if (debug) t.addData("x", Blob.getBoxFit().center.x);
            if (debug) t.addData("y", Blob.getBoxFit().center.y);
            if (debug) t.addData("Ex", ErrorX);
            if (debug) t.addData("Ey", ErrorY);


            return true;
        } else if (debug) t.addLine("NONE DETECTED");


        return false;
    }

    double filterAngle(RotatedRect box) {
        Point[] p = new Point[4];
        box.points(p);
        double angle;
        if (distanceSq(p[0], p[1]) > distanceSq(p[1], p[2])) {
            angle = Math.atan2((p[0].y - p[1].y), (p[0].x - p[1].x));
        } else {
            angle = Math.atan2((p[1].y - p[2].y), (p[1].x - p[2].x));
        }
        if (angle < 0) angle += 180;
        return angle;
    }


    double distanceSq(Point a, Point b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return dx * dx + dy * dy;
    }
}

