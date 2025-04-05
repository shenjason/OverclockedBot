package Overclocked.OpModes.Main.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.nio.file.Path;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name="AutoTest", group = "Test")
public class AutoTest extends OpMode {


    int pathState = 0;

    private final Pose start = new Pose(0, 0, Math.toRadians(180));

    private final Pose end = new Pose(10, 10, Math.toRadians(0));
    private PathChain testPath, returnPath;
    Follower follower;

    public void pathStateUpdate(){
        switch (pathState){
            case 0:
                follower.followPath(testPath);
                pathState = 1;
                break;
            case 1:
                follower.followPath(returnPath);
                pathState = -1;
                break;
        }
    }



    @Override
    public void init() {

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        follower.setStartingPose(start);

        testPath = follower.pathBuilder().addPath(new BezierLine(new Point(start), new Point(end))).setLinearHeadingInterpolation(start.getHeading(), end.getHeading()).build();
        returnPath = follower.pathBuilder().addPath(new BezierCurve(new Point(end), new Point(8, 8), new Point(start))).setLinearHeadingInterpolation(end.getHeading(), start.getHeading()).build();
    }

    @Override
    public void loop() {

        follower.update();
        pathStateUpdate();

//        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void start() {
        pathState =0;
    }
}
