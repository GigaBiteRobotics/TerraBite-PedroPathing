package pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import drive.RobotCoreCustom;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "Sample Square Auto", group = "Examples")
public class SampleSquare extends OpMode{
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private final Pose startPose = new Pose(72, 72, Math.toRadians(90));
    private final Pose pose0 = new Pose(72, 120, Math.toRadians(180));
    private final Pose pose1 = new Pose(120, 120, Math.toRadians(270));
    private final Pose pose2 = new Pose(120, 72, Math.toRadians(0));
    private PathChain line0chain, line1chain, line2chain, line3chain, endPathChain;
    private RobotCoreCustom robotCoreCustom = new RobotCoreCustom();
    public int pathState = 0;
    private boolean isRunning = false;
    private ElapsedTime endPathCentering = new ElapsedTime();
    int stopState = 0;


    public void buildPaths() {
        telemetry.addData("Path status", "Building");
        telemetry.update();
        line0chain = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(pose0)))
                .setLinearHeadingInterpolation(startPose.getHeading(), pose0.getHeading())
                .build();
        line1chain = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pose0), new Point(pose1)))
                .setLinearHeadingInterpolation(pose0.getHeading(), pose1.getHeading())
                .build();
        line2chain = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pose1), new Point(pose2)))
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .build();
        line3chain = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pose2), new Point(startPose)))
                .setLinearHeadingInterpolation(pose2.getHeading(), startPose.getHeading())
                .build();

        telemetry.addData("Path status", "Ready");
    }
    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                if (!isRunning) {
                    follower.followPath(line0chain);
                    isRunning = true;
                }
                if (robotCoreCustom.isPathFinished(follower, pose0)) {
                    pathState = 1;
                    isRunning = false;
                }
                break;
            case 1:
                if (!isRunning) {
                    follower.followPath(line1chain);
                    isRunning = true;
                }
                if (robotCoreCustom.isPathFinished(follower, pose1)) {
                    pathState = 2;
                    isRunning = false;
                }
                break;
            case 2:
                if (!isRunning) {
                    follower.followPath(line2chain);
                    isRunning = true;
                }
                if (robotCoreCustom.isPathFinished(follower, pose2)) {
                    pathState = 3;
                    isRunning = false;
                }
                break;
            case 3:
                if (!isRunning) {
                    follower.followPath(line3chain, true);
                    endPathCentering.reset();
                    isRunning = true;
                }
                if (robotCoreCustom.isPathFinished(follower, startPose)) {
                    switch (stopState) {
                        case 0:
                            endPathCentering.reset();
                            stopState = 1;
                            break;
                        case 1:
                            if (endPathCentering.milliseconds() > 1750) {
                                pathState = -1;
                                isRunning = false;
                                break;
                            }
                            break;

                    }

                }
                break;
            case -1:
                requestOpModeStop();
        }
    }
    @Override
    public void init() {
        robotCoreCustom.robotCoreInit(hardwareMap);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        follower.getDashboardPoseTracker();
    }
    @Override
    public void loop() {
        follower.update();
        autoPathUpdate();
        telemetry.addData("Path status", "Running");
        telemetry.addData("Path State", pathState);
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.update();
    }
}