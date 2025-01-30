package auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import drive.PersistentStorage;
import drive.RobotCoreCustom;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "OnlyParkAuto", group = "!", preselectTeleOp = "TerraBiteDrive V1")
public class ONLYparkAutoRight extends OpMode {

    private Follower follower;
    private RobotCoreCustom robotCoreCustom = new RobotCoreCustom();

    private int pathState = 6; // Tracks the current state of the autonomous routine

    // Define poses
    private final Pose startPose = new Pose(8.75, 65.75, 0);
    private final Pose toHuman0 = new Pose(10, 26.3, 0);

    private PathChain parkPC, hangPreloadPC, grabFromHuman0PC, hangOnSubPC, pushSamples0PC, pushSamples1PC, pushSamples2PC, pushSamples3PC;
    private GoBildaPinpointDriver odo;
    private ElapsedTime pathTimer = new ElapsedTime();

    public void buildPaths() {
        parkPC = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(toHuman0)))
                .setConstantHeadingInterpolation(0)
                .build();
    }

    public void autoUpdate() {
        switch (pathState) {
            case 6:
                    follower.followPath(parkPC, true);
                    pathState = 7;
                break;

            case 7:
                telemetry.addData("Path Complete", "All paths finished");
                break;

            default:
                telemetry.addData("Error", "Unexpected pathState: " + pathState);
                break;
        }
    }

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        buildPaths();
        pathTimer.reset();
    }

    @Override
    public void loop() {
        follower.update();
        autoUpdate();
        telemetry.addData("pathState", pathState);
        telemetry.addData("odo status", odo.getDeviceStatus());
        telemetry.update();
    }

    @Override
    public void stop() {
        PersistentStorage.currentPose = follower.getPose();
    }
}
