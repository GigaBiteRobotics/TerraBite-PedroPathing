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

import java.util.MissingFormatArgumentException;

import drive.PersistentStorage;
import drive.RobotCoreCustom;
import drive.opmode.MainDriveOpmode;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "MainAutoRight V3", group = "!", preselectTeleOp = "TerraBiteDrive V1")
public class MainAutoRightV3 extends OpMode {

	private Follower follower;
	private final RobotCoreCustom robotCoreCustom = new RobotCoreCustom();
	private int pathState; // Tracks the current state of the autonomous routine

	// Define poses
	private final Pose startPose = new Pose(8.2, 65.75, Math.toRadians(0));
	private final Pose pose0 = new Pose(38, 65, Math.toRadians(180)); //hang 0

	private final Pose pose1 = new Pose(35, 48, Math.toRadians(315));
	private final Pose pose2 = new Pose(59, 26, Math.toRadians(315));
	private final Pose pose3 = new Pose(21.1, 24, Math.toRadians(270));
	private final Pose pose4 = new Pose(57.8, 19, Math.toRadians(270)); //Heavy Curve out left
	private final Pose pose5 = new Pose(21, 20, Math.toRadians(270));
	private final Pose pose6 = new Pose(56.5, 12.5, Math.toRadians(270));
	private final Pose pose7 = new Pose(21, 12.5, Math.toRadians(270));
	private final Pose pose8 = new Pose(18, 34.6, Math.toRadians(180)); //pickup Position
	private final Pose pose9 = new Pose(40.3, 67, Math.toRadians(180)); //hang 1
	private final Pose pose10 = new Pose(40.3, 70, Math.toRadians(180)); //hang 2
	private final Pose pose11 = new Pose(40.3, 72, Math.toRadians(180)); //hang 3
	private final Pose pose12 = new Pose(40.3, 75, Math.toRadians(180)); //hang 4
	private final Pose park = new Pose(15, 16, Math.toRadians(90));
	private PathChain hangPreloadPC, pushSamplesPC, hang1PC, hang2PC, hang3PC, hang4PC, parkPC, path2PC, path3PC, path4PC, path5PC, path6PC, path7PC, path8PC, backPC;
	private GoBildaPinpointDriver odo;
	private final ElapsedTime pathTimer = new ElapsedTime();
	double gripperRotationPosTarget = 0.5;
	MainDriveOpmode.gripperPos gripperTracking = MainDriveOpmode.gripperPos.CLOSE;
	MainDriveOpmode.gripperPitchPos gripperPitchTracking = MainDriveOpmode.gripperPitchPos.FORWARD;
	double gripperPitchPosTarget = 1;
	ElapsedTime gripperTimer = new ElapsedTime();
	int[] targetArmPos = {70, 70};
	int[] armDownPos = {70, 70};
	int armState = -1;
	int gripperGrabSamplePos = 0;
	MainDriveOpmode.PosWait downPosTracking = MainDriveOpmode.PosWait.WAIT;
	MainDriveOpmode.PosWait upPosTracking = MainDriveOpmode.PosWait.WAIT;
	MainDriveOpmode.targetSetPos targetSetPosTracking = MainDriveOpmode.targetSetPos.NA;
	ElapsedTime scoreTimer = new ElapsedTime();


	public void buildPaths() {
		hangPreloadPC = follower.pathBuilder()
				.addPath(new BezierLine(new Point(startPose), new Point(pose0)))
				.setConstantHeadingInterpolation(Math.toRadians(180))
				.build();
		pushSamplesPC = follower.pathBuilder()
				.addPath(new BezierLine(new Point(pose0), new Point(pose1)))
				.setLinearHeadingInterpolation(pose0.getHeading(), pose1.getHeading())
				.build();

		path2PC = follower.pathBuilder()
				.addPath(new BezierLine(new Point(pose1), new Point(pose2)))
				.setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
				.build();

		path3PC = follower.pathBuilder()
				.addPath(new BezierCurve(new Point(pose2), new Point(new Pose(56.3, 32.8, Math.toRadians(315))), new Point(pose3)))
				.setConstantHeadingInterpolation(Math.toRadians(270))
				.build();

		path4PC = follower.pathBuilder()
				.addPath(new BezierCurve(new Point(pose3),new Point(new Pose(66, 32, Math.toRadians(270))), new Point(pose4)))
				.setLinearHeadingInterpolation(pose3.getHeading(), pose4.getHeading())
				.build();

		path5PC = follower.pathBuilder()
				.addPath(new BezierLine(new Point(pose4), new Point(pose5)))
				.setLinearHeadingInterpolation(pose4.getHeading(), pose5.getHeading())
				.build();

		path6PC = follower.pathBuilder()
				.addPath(new BezierCurve(new Point(pose5),new Point(66, 32) , new Point(pose6)))
				.setLinearHeadingInterpolation(pose5.getHeading(), pose6.getHeading()) // Needs Curve
				.build();

		path7PC = follower.pathBuilder()
				.addPath(new BezierLine(new Point(pose6), new Point(pose7)))
				.setLinearHeadingInterpolation(pose6.getHeading(), pose7.getHeading())
				.build();

		path8PC = follower.pathBuilder()
				.addPath(new BezierCurve(new Point(pose7),new Point(31, 27), new Point(pose8)))
				.setLinearHeadingInterpolation(pose7.getHeading(), pose8.getHeading())
				.build();

		hang1PC = follower.pathBuilder()
				.addPath(new BezierCurve(new Point(pose8),new Point(24, 60), new Point(pose9)))
				.setConstantHeadingInterpolation(Math.toRadians(180))
				.build();

		hang2PC = follower.pathBuilder()
				.addPath(new BezierCurve(new Point(pose8),new Point(24, 60), new Point(pose10)))
				.setConstantHeadingInterpolation(Math.toRadians(180))
				.build();

		hang3PC = follower.pathBuilder()
				.addPath(new BezierCurve(new Point(pose8),new Point(24, 60), new Point(pose11)))
				.setConstantHeadingInterpolation(Math.toRadians(180))
				.build();

		hang4PC = follower.pathBuilder()
				.addPath(new BezierCurve(new Point(pose8),new Point(24, 60), new Point(pose12)))
				.setConstantHeadingInterpolation(Math.toRadians(180))
				.build();
		backPC = follower.pathBuilder()
				.addPath(new BezierLine(new Point(pose12), new Point(pose8)))
				.setConstantHeadingInterpolation(Math.toRadians(180))
				.build();

		parkPC = follower.pathBuilder()
				.addPath(new BezierLine(new Point(pose12), new Point(park)))
				.setLinearHeadingInterpolation(pose12.getHeading(), park.getHeading())
				.build();
	}
	public void autoUpdate() {
		switch (pathState) {
			case 0:
				follower.followPath(hangPreloadPC, true);
				pathState = 1;
				armState = 0;
				upPosTracking = MainDriveOpmode.PosWait.WAIT;
				targetSetPosTracking = MainDriveOpmode.targetSetPos.UP;
				break;

			case 1:
				if (robotCoreCustom.isPathFinished(follower, pose0) && armState == 0) {
					armState = 1;
					follower.followPath(pushSamplesPC, true);
					upPosTracking = MainDriveOpmode.PosWait.WAIT;
					targetSetPosTracking = MainDriveOpmode.targetSetPos.SCORE;
					pathTimer.reset();
					pathState = 2;
				}
				break;
			case 2:
				if (robotCoreCustom.isPathFinished(follower, pose1)) {
					follower.followPath(path2PC, true);
					pathState = 3;
				}
				break;
			case 3:
				if (robotCoreCustom.isPathFinished(follower, pose2)) {
					follower.followPath(path3PC, true);
					pathState = 4;
				}
				break;

			case 4:
				if (robotCoreCustom.isPathFinished(follower, pose3)) {
					follower.followPath(path4PC, true);
					pathState = 5;
				}
				break;

			case 5:
				if (robotCoreCustom.isPathFinished(follower, pose4)) {
					follower.followPath(path5PC, true);
					pathState = 6;
				}
				break;

			case 6:
				if (robotCoreCustom.isPathFinished(follower, pose5)) {
					follower.followPath(path6PC, true);
					pathState = 7;
				}
				break;

			case 7:
				if (robotCoreCustom.isPathFinished(follower, pose6)) {
					follower.followPath(path7PC, true);
					gripperTracking = MainDriveOpmode.gripperPos.OPEN;
					pathState = 8;
				}
				break;

			case 8:
				if (robotCoreCustom.isPathFinished(follower, pose7)) {
					follower.followPath(path8PC, true);
					downPosTracking = MainDriveOpmode.PosWait.WAIT;
					targetSetPosTracking = MainDriveOpmode.targetSetPos.DOWN;
					pathState = 9;
					pathTimer.reset();
				}
				break;

			case 9:
				if (robotCoreCustom.isPathFinished(follower, pose8)) {
					gripperTracking = MainDriveOpmode.gripperPos.CLOSE;
				}
				if (robotCoreCustom.isPathFinished(follower, pose8) && pathTimer.milliseconds() > 2000) {
					follower.followPath(hang1PC, true);
					upPosTracking = MainDriveOpmode.PosWait.WAIT;
					targetSetPosTracking = MainDriveOpmode.targetSetPos.UP;
					pathState = 10;
				}
				break;

			case 10:
				if (robotCoreCustom.isPathFinished(follower, pose9)) {
					upPosTracking = MainDriveOpmode.PosWait.WAIT;
					targetSetPosTracking = MainDriveOpmode.targetSetPos.SCORE;
					armState = 1;
					follower.followPath(backPC, true);
					pathTimer.reset();
				}
				break;

			case 11:
				if (pathTimer.milliseconds() > 500) {
					downPosTracking = MainDriveOpmode.PosWait.WAIT;
					targetSetPosTracking = MainDriveOpmode.targetSetPos.DOWN;
					gripperTracking = MainDriveOpmode.gripperPos.OPEN;
				}
				if (robotCoreCustom.isPathFinished(follower, pose8) && pathTimer.milliseconds() > 2000) {
					upPosTracking = MainDriveOpmode.PosWait.WAIT;
					targetSetPosTracking = MainDriveOpmode.targetSetPos.UP;
					follower.followPath(hang2PC, true);
					pathState = 12;
					pathTimer.reset();
				}
				break;

			case 12:
				if (robotCoreCustom.isPathFinished(follower, pose10)) {
					upPosTracking = MainDriveOpmode.PosWait.WAIT;
					targetSetPosTracking = MainDriveOpmode.targetSetPos.SCORE;
				armState = 1;
				follower.followPath(backPC, true);
			}
				break;

			case 13:
				if (pathTimer.milliseconds() > 500) {
					downPosTracking = MainDriveOpmode.PosWait.WAIT;
					targetSetPosTracking = MainDriveOpmode.targetSetPos.DOWN;
					gripperTracking = MainDriveOpmode.gripperPos.OPEN;
				}
				if (robotCoreCustom.isPathFinished(follower, pose8) && pathTimer.milliseconds() > 2000) {
					upPosTracking = MainDriveOpmode.PosWait.WAIT;
					targetSetPosTracking = MainDriveOpmode.targetSetPos.UP;
					follower.followPath(hang3PC, true);
					pathState = 14;
				}
				break;

			case 14: // hang
				if (robotCoreCustom.isPathFinished(follower, pose11)) {
					upPosTracking = MainDriveOpmode.PosWait.WAIT;
					targetSetPosTracking = MainDriveOpmode.targetSetPos.SCORE;
					armState = 1;
					follower.followPath(backPC, true);
					pathTimer.reset();
				}
				break;

			case 15:
				if (pathTimer.milliseconds() > 500) {
					downPosTracking = MainDriveOpmode.PosWait.WAIT;
					targetSetPosTracking = MainDriveOpmode.targetSetPos.DOWN;
					gripperTracking = MainDriveOpmode.gripperPos.OPEN;
				}
				if (robotCoreCustom.isPathFinished(follower, pose8) && pathTimer.milliseconds() > 2000) {
					upPosTracking = MainDriveOpmode.PosWait.WAIT;
					targetSetPosTracking = MainDriveOpmode.targetSetPos.UP;
					follower.followPath(hang4PC, true);
					pathState = 16;
				}
				break;

			case 16: // hang
				if (robotCoreCustom.isPathFinished(follower, pose12) && armState == 0) {
					armState = 1;
					follower.followPath(backPC, true);
					pathTimer.reset();
				}
				if (armState == 1) {
					pathState = 17;
					armState = 0;
				}
				break;

			case 17: // park
				if (robotCoreCustom.isPathFinished(follower, pose8)) {
					follower.followPath(parkPC, true);
					pathState = -1;
				}
				break;
			case -1:
				telemetry.addData("Auto Complete", "All paths finished");
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
		robotCoreCustom.robotCoreInit(hardwareMap);
		robotCoreCustom.homeExt();
		robotCoreCustom.homeRot();
		robotCoreCustom.gripperPitch.setPosition(0.50);
	}

	@Override
	public void init_loop() {
		robotCoreCustom.updateAll();
		targetArmPos = new int[]{70, 100};
		gripperChecking();
	}

	@Override
	public void loop() {
		follower.update();
		upArmUpdate();
		scoreUpdate();
		downPosUpdate();
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
		telemetry.update();
	}

	@Override
	public void stop() {
		PersistentStorage.currentPose = follower.getPose();
	}
	public void gripperChecking() {
		// Update hardware states based on tracking
		robotCoreCustom.setGripperPitch(gripperPitchPosTarget);

		if (gripperTracking == MainDriveOpmode.gripperPos.OPEN) robotCoreCustom.setGripper(0.65);
		if (gripperTracking == MainDriveOpmode.gripperPos.CLOSE) robotCoreCustom.setGripper(0.46);
		if (gripperTracking == MainDriveOpmode.gripperPos.LIGHT) robotCoreCustom.setGripper(0.5);
		robotCoreCustom.setGripperRotation(gripperRotationPosTarget);
	}
	public void scoreUpdate() {
		if (downPosTracking == MainDriveOpmode.PosWait.WAIT && targetSetPosTracking == MainDriveOpmode.targetSetPos.SCORE) {
			targetArmPos[0] = (int) (1650);
			targetArmPos[1] = (int) (800);
			gripperPitchPosTarget = 0.6;
			scoreTimer.reset();
			downPosTracking = MainDriveOpmode.PosWait.GO;
		}
		if (downPosTracking == MainDriveOpmode.PosWait.GO && targetSetPosTracking == MainDriveOpmode.targetSetPos.SCORE && scoreTimer.milliseconds() > 375) {
			gripperTracking = MainDriveOpmode.gripperPos.OPEN;
			targetSetPosTracking = MainDriveOpmode.targetSetPos.NA;
		}
	}
	public void upArmUpdate() {
		if (upPosTracking == MainDriveOpmode.PosWait.WAIT && targetSetPosTracking == MainDriveOpmode.targetSetPos.UP) {
			targetArmPos[0] = (int) (2250);
			targetArmPos[1] = (int) (10);
			gripperPitchPosTarget = 0.3;
			upPosTracking = MainDriveOpmode.PosWait.GO;
			gripperTracking = MainDriveOpmode.gripperPos.LIGHT;
			gripperTimer.reset();
		}
		if (upPosTracking == MainDriveOpmode.PosWait.GO && targetSetPosTracking == MainDriveOpmode.targetSetPos.UP && gripperTimer.milliseconds() > 750) {
			gripperTracking = MainDriveOpmode.gripperPos.CLOSE;
			targetSetPosTracking = MainDriveOpmode.targetSetPos.NA;
		}
	}
	public void downPosUpdate() {
		if (downPosTracking == MainDriveOpmode.PosWait.WAIT && targetSetPosTracking == MainDriveOpmode.targetSetPos.DOWN) {
			targetArmPos[1] = (int) (240);
			targetArmPos[0] = (int) (330);
			gripperPitchPosTarget = 0.5;
			gripperTracking = MainDriveOpmode.gripperPos.OPEN;
		}
		if (downPosTracking == MainDriveOpmode.PosWait.GO && targetSetPosTracking == MainDriveOpmode.targetSetPos.DOWN) {
			targetArmPos[1] = (int) (240);
			targetArmPos[0] = (int) (330);
			gripperPitchPosTarget = 0.5;
			gripperTracking = MainDriveOpmode.gripperPos.CLOSE;
		}
	}

}
