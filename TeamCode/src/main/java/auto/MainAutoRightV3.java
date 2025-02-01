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
	private final Pose pose0 = new Pose(36, 65, Math.toRadians(180)); //hang 0
	private final Pose pose1 = new Pose(35, 48, Math.toRadians(315));
	private final Pose pose2 = new Pose(59, 26, Math.toRadians(315));
	private final Pose pose3 = new Pose(21.1, 24, Math.toRadians(270));
	private final Pose pose4 = new Pose(57.8, 19, Math.toRadians(270)); //Heavy Curve out left
	private final Pose pose5 = new Pose(21, 20, Math.toRadians(270));
	private final Pose pose6 = new Pose(56.5, 12.5, Math.toRadians(270));
	private final Pose pose7 = new Pose(21, 12.5, Math.toRadians(270));
	private final Pose pose8 = new Pose(13, 34.6, Math.toRadians(180)); //pickup Position
	private final Pose pose9 = new Pose(40.3, 67, Math.toRadians(180)); //hang 1
	private final Pose pose10 = new Pose(40.3, 70, Math.toRadians(180)); //hang 2
	private final Pose pose11 = new Pose(40.3, 72, Math.toRadians(180)); //hang 3
	private final Pose pose12 = new Pose(40.3, 75, Math.toRadians(180)); //hang 4
	private final Pose park = new Pose(15, 16, Math.toRadians(90));
	private PathChain hangPreloadPC, pushSamplesPC, hang1PC, hang2PC, hang3PC, hang4PC, parkPC, path2PC, path3PC, path4PC, path5PC, path6PC, path7PC, path8PC, backPC;
	private GoBildaPinpointDriver odo;
	private final ElapsedTime pathTimer = new ElapsedTime();
	MainDriveOpmode.gripperRollerPos gripperRollerTracking = MainDriveOpmode.gripperRollerPos.STOPPED;
	MainDriveOpmode.gripperPos gripperTracking = MainDriveOpmode.gripperPos.CLOSE;
	MainDriveOpmode.gripperPitchPos gripperPitchTracking = MainDriveOpmode.gripperPitchPos.FORWARD;
	ElapsedTime gripperTimer;
	int[] targetArmPos = {70, 0};
	int[] armDownPos = {70, 70};
	int[] armMidPos = {300, 70};
	int[] armUpPos = {2600, 540};
	int[] armFirstUpPos = {2600, 580};
	int[] armScorePos = {2000, 630};
	int armState = -1;

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
				.addPath(new BezierLine(new Point(pose8), new Point(pose9)))
				.setConstantHeadingInterpolation(Math.toRadians(180))
				.build();

		hang2PC = follower.pathBuilder()
				.addPath(new BezierLine(new Point(pose8), new Point(pose10)))
				.setConstantHeadingInterpolation(Math.toRadians(180))
				.build();

		hang3PC = follower.pathBuilder()
				.addPath(new BezierLine(new Point(pose8), new Point(pose11)))
				.setConstantHeadingInterpolation(Math.toRadians(180))
				.build();

		hang4PC = follower.pathBuilder()
				.addPath(new BezierLine(new Point(pose8), new Point(pose12)))
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
				targetArmPos = armFirstUpPos;
				robotCoreCustom.gripperPitch.setPosition(0.6);
				pathState = 1;
				armState = 0;
				break;

			case 1:
				if (robotCoreCustom.isPathFinished(follower, pose0) && robotCoreCustom.motorControllerRot.motor.getCurrentPosition() > 2450 && robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() > 545 && armState == 0) {
					robotCoreCustom.gripperPitch.setPosition(0.56);
					armState = 1;
					follower.followPath(pushSamplesPC, true);
					targetArmPos = armScorePos;
					pathTimer.reset();
				}
				if (armState == 1  && robotCoreCustom.motorControllerRot.motor.getCurrentPosition() < 2300 && robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() > 600) {
					pathState = 2;
					armState = 0;
				}
				break;

			case 2:
				if (robotCoreCustom.isPathFinished(follower, pose1)) {
					targetArmPos = armDownPos;
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
					robotCoreCustom.gripperPitch.setPosition(0.7);
					gripperTracking = MainDriveOpmode.gripperPos.OPEN;
				}
				break;

			case 7:
				if (robotCoreCustom.isPathFinished(follower, pose6)) {
					follower.followPath(path7PC, true);
					pathState = 8;
				}
				break;

			case 8:
				if (robotCoreCustom.isPathFinished(follower, pose7)) {
					follower.followPath(path8PC, true);
					gripperTracking = MainDriveOpmode.gripperPos.OPEN;
					pathState = 9;
				}
				break;

			case 9:
				if (robotCoreCustom.isPathFinished(follower, pose8)) {
					gripperTracking = MainDriveOpmode.gripperPos.CLOSE;
					targetArmPos = armUpPos;
					follower.followPath(hang1PC, true);
					pathState = 10;
					armState = 0;
				}
				break;

			case 10:
				if (robotCoreCustom.isPathFinished(follower, pose9) && robotCoreCustom.motorControllerRot.motor.getCurrentPosition() > 2450 && robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() > 520 && armState == 0) {
					robotCoreCustom.gripperPitch.setPosition(0.56);
					armState = 1;
					follower.followPath(backPC, true);
					targetArmPos = armScorePos;
					pathTimer.reset();
				}
				if (armState == 1  && robotCoreCustom.motorControllerRot.motor.getCurrentPosition() < 2300 && robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() > 600) {
					pathState = 11;
					armState = 0;
					targetArmPos = armDownPos;
					gripperTracking = MainDriveOpmode.gripperPos.OPEN;
				}
				break;

			case 11:
				if (robotCoreCustom.isPathFinished(follower, pose8)) {
					follower.followPath(hang2PC, true);
					gripperTracking = MainDriveOpmode.gripperPos.CLOSE;
					targetArmPos = armUpPos;
					robotCoreCustom.gripperPitch.setPosition(0.7);
					pathState = 12;
				}
				break;

			case 12:
				if (robotCoreCustom.isPathFinished(follower, pose10) && robotCoreCustom.motorControllerRot.motor.getCurrentPosition() > 2450 && robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() > 500 && armState == 0) {
					robotCoreCustom.gripperPitch.setPosition(0.56);
					armState = 1;
					follower.followPath(backPC, true);
					targetArmPos = armScorePos;
					pathTimer.reset();
				}
				if (armState == 1  && robotCoreCustom.motorControllerRot.motor.getCurrentPosition() < 2300 && robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() > 600) {
					pathState = 13;
					armState = 0;
					targetArmPos = armDownPos;
					gripperTracking = MainDriveOpmode.gripperPos.OPEN;
				}
				break;

			case 13:
				if (robotCoreCustom.isPathFinished(follower, pose8)) {
					follower.followPath(hang3PC, true);
					gripperTracking = MainDriveOpmode.gripperPos.CLOSE;
					targetArmPos = armUpPos;
					robotCoreCustom.gripperPitch.setPosition(0.7);
					pathState = 14;
				}
				break;

			case 14:
				if (robotCoreCustom.isPathFinished(follower, pose11) && robotCoreCustom.motorControllerRot.motor.getCurrentPosition() > 2450 && robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() > 500 && armState == 0) {
					robotCoreCustom.gripperPitch.setPosition(0.56);
					armState = 1;
					follower.followPath(backPC, true);
					targetArmPos = armScorePos;
					pathTimer.reset();
				}
				if (armState == 1  && robotCoreCustom.motorControllerRot.motor.getCurrentPosition() < 2300 && robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() > 600) {
					pathState = 15;
					armState = 0;
					targetArmPos = armDownPos;
					gripperTracking = MainDriveOpmode.gripperPos.OPEN;
				}
				break;

			case 15:
				if (robotCoreCustom.isPathFinished(follower, pose8)) {
					follower.followPath(hang4PC, true);
					gripperTracking = MainDriveOpmode.gripperPos.CLOSE;
					targetArmPos = armUpPos;
					robotCoreCustom.gripperPitch.setPosition(0.7);
					pathState = 16;
				}
				break;

			case 16:
				if (robotCoreCustom.isPathFinished(follower, pose12) && robotCoreCustom.motorControllerRot.motor.getCurrentPosition() > 2450 && robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() > 500 && armState == 0) {
					robotCoreCustom.gripperPitch.setPosition(0.56);
					armState = 1;
					follower.followPath(backPC, true);
					targetArmPos = armScorePos;
					pathTimer.reset();
				}
				if (armState == 1  && robotCoreCustom.motorControllerRot.motor.getCurrentPosition() < 2300 && robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() > 600) {
					pathState = 17;
					armState = 0;
					targetArmPos = armDownPos;
					gripperTracking = MainDriveOpmode.gripperPos.OPEN;
					robotCoreCustom.gripperPitch.setPosition(0.7);
				}
				break;

			case 17:
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
