package auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import drive.PersistentStorage;
import drive.RobotCoreCustom;
import drive.opmode.MainDriveOpmode;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "MainAutoLeft V1", group = "!", preselectTeleOp = "TerraBiteDrive V1")
public class MainAutoLeftV1 extends OpMode {

	private Follower follower;
	private final RobotCoreCustom robotCoreCustom = new RobotCoreCustom();
	private int pathState; // Tracks the current state of the autonomous routine

	// Define poses
	private final Pose startPose = new Pose(7.5, 88.6, Math.toRadians(0));
	private final Pose pose0 = new Pose(36, 77, Math.toRadians(180)); // hang specimen & start down arm pos
	private final Pose pose1 = new Pose(20, 120, Math.toRadians(0)); // wait for down arm pos & start intake motors
	private final Pose pose2 = new Pose(25, 120, Math.toRadians(0)); // once at pos close gripper & stop intake motors & start up position
	private final Pose pose3 = new Pose(12.6, 131.7, Math.toRadians(300)); // wait for up position the gripper pitch forward to back then open gripper for drop, then down arm pos & forward gripper pos and open gripper + intake motors.
	private final Pose pose4 = new Pose(20, 130, Math.toRadians(0)); // wait for down arm pos & start intake motors
	private final Pose pose5 = new Pose(30, 130, Math.toRadians(0)); // once at pos close gripper & stop intake motors & start up position
	private final Pose pose6 = new Pose(12.6, 131.7, Math.toRadians(300)); // wait for up position the gripper pitch forward to back then open gripper for drop, then down arm pos & forward gripper pos and open gripper + intake motors.
	private final Pose pose7 = new Pose(22, 128, 0.5333); // wait for down arm pos & start intake motors
	private final Pose pose8 = new Pose(26, 133, 0.2557); // once at pos close gripper & stop intake motors & start up position
	private final Pose pose9 = new Pose(12.6, 131.7, Math.toRadians(300)); // wait for up position the gripper pitch forward to back then open gripper for drop, then down arm pos & forward gripper pos and open gripper + intake motors.
	private final Pose pose10 = new Pose(78, 101, 2.5725); // park pos
	public PathChain pc0, pc1, pc2, pc3, pc4, pc5, pc6, pc7, pc8, pc9, pc10;
	private GoBildaPinpointDriver odo;
	private final ElapsedTime pathTimer = new ElapsedTime();
	private final ElapsedTime elapsedTime = new ElapsedTime();
	MainDriveOpmode.gripperRollerPos gripperRollerTracking = MainDriveOpmode.gripperRollerPos.STOPPED;
	MainDriveOpmode.gripperPos gripperTracking = MainDriveOpmode.gripperPos.CLOSE;
	MainDriveOpmode.gripperPitchPos gripperPitchTracking = MainDriveOpmode.gripperPitchPos.FORWARD;
	ElapsedTime gripperTimer;
	int[] targetArmPos = {70, 20};
	int[] armDownPos = {180, 1800};
	int[] armUpPos = {540, 2600};
	int[] armFirstUpPos = {2600, 580};
	int[] armScorePos = {2000, 630};
	int armState = -1;

	public void buildPaths() {
		pc0 = follower.pathBuilder()
				.addPath(new BezierLine(new Point(startPose), new Point(pose0)))
				.setLinearHeadingInterpolation(startPose.getHeading(), pose0.getHeading())
				.setPathEndVelocityConstraint(5)
				.build();

		pc1 = follower.pathBuilder()
				.addPath(new BezierLine(new Point(pose0), new Point(pose1)))
				.setLinearHeadingInterpolation(pose0.getHeading(), pose1.getHeading())
				.setPathEndVelocityConstraint(5)
				.build();

		pc2 = follower.pathBuilder()
				.addPath(new BezierLine(new Point(pose1), new Point(pose2)))
				.setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
				.setPathEndVelocityConstraint(5)
				.build();

		pc3 = follower.pathBuilder()
				.addPath(new BezierLine(new Point(pose2), new Point(pose3)))
				.setLinearHeadingInterpolation(pose2.getHeading(), pose3.getHeading())
				.setPathEndVelocityConstraint(5)
				.build();

		pc4 = follower.pathBuilder()
				.addPath(new BezierLine(new Point(pose3), new Point(pose4)))
				.setLinearHeadingInterpolation(pose3.getHeading(), pose4.getHeading())
				.setPathEndVelocityConstraint(5)
				.build();

		pc5 = follower.pathBuilder()
				.addPath(new BezierLine(new Point(pose4), new Point(pose5)))
				.setLinearHeadingInterpolation(pose4.getHeading(), pose5.getHeading())
				.setPathEndVelocityConstraint(5)
				.build();

		pc6 = follower.pathBuilder()
				.addPath(new BezierLine(new Point(pose5), new Point(pose6)))
				.setLinearHeadingInterpolation(pose5.getHeading(), pose6.getHeading())
				.setPathEndVelocityConstraint(5)
				.build();

		pc7 = follower.pathBuilder()
				.addPath(new BezierLine(new Point(pose6), new Point(pose7)))
				.setLinearHeadingInterpolation(pose6.getHeading(), pose7.getHeading())
				.setPathEndVelocityConstraint(5)
				.build();

		pc8 = follower.pathBuilder()
				.addPath(new BezierLine(new Point(pose7), new Point(pose8)))
				.setLinearHeadingInterpolation(pose7.getHeading(), pose8.getHeading())
				.setPathEndVelocityConstraint(5)
				.build();

		pc9 = follower.pathBuilder()
				.addPath(new BezierLine(new Point(pose8), new Point(pose9)))
				.setLinearHeadingInterpolation(pose8.getHeading(), pose9.getHeading())
				.setPathEndVelocityConstraint(5)
				.build();

		pc10 = follower.pathBuilder()
				.addPath(new BezierLine(new Point(pose9), new Point(pose10)))
				.setLinearHeadingInterpolation(pose9.getHeading(), pose10.getHeading())
				.setPathEndVelocityConstraint(5)
				.build();
	}
	public void autoUpdate() {
		switch (pathState) {
			case 0:
				follower.followPath(pc0, true);
				targetArmPos = armFirstUpPos;
				robotCoreCustom.gripperPitch.setPosition(0.6);
				pathState = 1;
				armState = 0;
				break;
			case 1:
				if (robotCoreCustom.isPathFinished(follower, pose0) && robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() > 540 && robotCoreCustom.motorControllerRot.motor.getCurrentPosition() > 2450 && armState == 0) {
					robotCoreCustom.gripperPitch.setPosition(0.56);
					armState = 1;
					follower.followPath(pc1, true);
					targetArmPos = armDownPos;
					pathTimer.reset();
				}
				if (armState == 1) {
					pathState = 2;
					armState = 0;
					targetArmPos = armDownPos;
					gripperTracking = MainDriveOpmode.gripperPos.OPEN;
					gripperRollerTracking = MainDriveOpmode.gripperRollerPos.REVERSE;
					robotCoreCustom.gripperPitch.setPosition(0.15);
					targetArmPos = armDownPos;
				}
				break;
			case 2:
				if (robotCoreCustom.isPathFinished(follower, pose1) && robotCoreCustom.motorControllerRot.motor.getCurrentPosition() < 190 && robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() > 1700 && armState == 0) {
					follower.followPath(pc2, true);
					pathState = 3;
					armState = 0;
				}
				break;
			case 3:
				if (robotCoreCustom.isPathFinished(follower, pose2)) {
					follower.followPath(pc3,true);
					pathState = 4;
					armState = 0;
					gripperTracking = MainDriveOpmode.gripperPos.CLOSE;
					targetArmPos = armUpPos;
				}
				break;
			case 4:
				if (robotCoreCustom.isPathFinished(follower, pose3)) {  // && robotCoreCustom.motorControllerRot.motor.getCurrentPosition() > 2450 && robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() > 530 && armState == 0) {
					armState = 1;
					gripperRollerTracking = MainDriveOpmode.gripperRollerPos.STOPPED;
					gripperTracking = MainDriveOpmode.gripperPos.OPEN;
					robotCoreCustom.gripperPitch.setPosition(0.6);
					follower.followPath(pc4, true);
					elapsedTime.reset();
				}
				break;
			case 5:
				if (robotCoreCustom.isPathFinished(follower, pose4)) {
					follower.followPath(pc5, true);
					pathState = 6;
					armState = 0;
					gripperRollerTracking = MainDriveOpmode.gripperRollerPos.STOPPED;
					gripperTracking = MainDriveOpmode.gripperPos.CLOSE;
					targetArmPos = armUpPos;
				}
				break;
			case 6:
				if (robotCoreCustom.isPathFinished(follower, pose5) && robotCoreCustom.motorControllerRot.motor.getCurrentPosition() > 2450 && robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() > 530 && armState == 0) {
					armState = 1;
					gripperRollerTracking = MainDriveOpmode.gripperRollerPos.STOPPED;
					gripperTracking = MainDriveOpmode.gripperPos.CLOSE;
					robotCoreCustom.gripperPitch.setPosition(0.6);
					elapsedTime.reset();
				}
				if (elapsedTime.milliseconds() > 200 && armState == 1) {
					armState = 0;
					follower.followPath(pc6, true);
					pathState = 7;
					targetArmPos = armDownPos;
					gripperRollerTracking = MainDriveOpmode.gripperRollerPos.STOPPED;
					robotCoreCustom.gripperPitch.setPosition(0.15);
					gripperTracking = MainDriveOpmode.gripperPos.OPEN;
				}
				break;
			default:
				telemetry.addData("Path state not found", pathState);
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
		robotCoreCustom.robotCoreInit(hardwareMap);
		robotCoreCustom.homeExt();
		robotCoreCustom.homeRot();
		robotCoreCustom.gripperPitch.setPosition(0.50);
	}

	@Override
	public void init_loop() {
		robotCoreCustom.updateAll();
		gripperChecking();
	}

	@Override
	public void loop() {
		follower.update();
		autoUpdate();
		robotCoreCustom.updateAll();
		robotCoreCustom.updateHoming();
		robotCoreCustom.setArmPos(targetArmPos);
		gripperChecking();
		telemetry.addData("pathState", pathState);
		telemetry.addData("odo status", odo.getDeviceStatus());
		telemetry.addData("time", pathTimer.time());
		telemetry.addData("armState", armState);
		telemetry.addData("armExtPos", robotCoreCustom.motorControllerExt0.motor.getCurrentPosition());
		telemetry.addData("armRotPos", robotCoreCustom.motorControllerRot.motor.getCurrentPosition());
		telemetry.addData("armExtTarget", targetArmPos[1]);
		telemetry.addData("armRotTarget", targetArmPos[0]);
		telemetry.update();
	}

	@Override
	public void stop() {
		PersistentStorage.currentPose = follower.getPose();
	}
	public void gripperChecking() {
		// Update hardware states based on tracking
		if (gripperTracking == MainDriveOpmode.gripperPos.CLOSE) {
			gripperRollerTracking = MainDriveOpmode.gripperRollerPos.STOPPED;
		}
		robotCoreCustom.setGripperRollers(
				(gripperRollerTracking == MainDriveOpmode.gripperRollerPos.STOPPED) ? 0.5 : 1,
				(gripperRollerTracking == MainDriveOpmode.gripperRollerPos.REVERSE) ? RobotCoreCustom.Direction.REVERSE : RobotCoreCustom.Direction.FORWARD
		);

        /*
        robotCoreCustom.setGripperPitch(
                (gripperPitchTracking == MainDriveOpmode.gripperPitchPos.FORWARD) ? 0.57 : 0.15
                // Adjust pitch as needed
        );

         */

		robotCoreCustom.setGripper(
				(gripperTracking == MainDriveOpmode.gripperPos.OPEN) ? 0.25 : 0.47   // Adjust positions for open/close
		);
	}

}
