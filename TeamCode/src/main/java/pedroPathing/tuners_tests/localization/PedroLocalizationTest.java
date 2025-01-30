package pedroPathing.tuners_tests.localization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import drive.RobotCoreCustom;
import drive.opmode.MainDriveOpmode;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(group = "Localization", name = "Pedro Localization Test")
public class PedroLocalizationTest extends OpMode {
    private DashboardPoseTracker dashboardPoseTracker;
    private Follower follower;
    private Telemetry telemetryA;
    private RobotCoreCustom robotCoreCustom = new RobotCoreCustom();
    double[] targetArmPos = {0, 0}; // Arm position: {rotation, extension}
    double[] targetArmVel = {0, 0}; // Arm velocity: {rotation, extension}
    ElapsedTime gamePadPollingRate = new ElapsedTime();
    int pollingRateMS = 5; // Polling rate for gamepad input
    boolean canExtend, canRetract, canTurnPositive, canTurnNegative = true; // Arm limits
    public enum PosWait {
        WAIT,
        WAIT1,
        GO
    }
    public enum targetSetPos {
        DOWN,
        UP,
        NA,
        VEL
    }
    MainDriveOpmode.PosWait downPosTracking;
    MainDriveOpmode.PosWait upPosTracking;
    MainDriveOpmode.targetSetPos targetSetPosTracking = MainDriveOpmode.targetSetPos.NA;


    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
        follower.setStartingPose(new Pose(8.75, 65.75, Math.toRadians(0)));
        robotCoreCustom.robotCoreInit(hardwareMap);
        robotCoreCustom.homeRot();
        robotCoreCustom.homeExt();
    }
    @Override
    public void init_loop() {
        robotCoreCustom.updateAll();
    }

    @Override
    public void loop() {
        follower.update();
        robotCoreCustom.updateAll();
        gamePadCont2DPos(gamepad2);
        telemetryA.addData("x", follower.getPose().getX());
        telemetryA.addData("y", follower.getPose().getY());
        telemetryA.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetryA.addData("extArmPos", robotCoreCustom.motorControllerExt0.motor.getCurrentPosition());
        telemetryA.addData("rotArmPos", robotCoreCustom.motorControllerRot.motor.getCurrentPosition());
        telemetryA.update();

        double calculatedPoseX = (follower.getPose().getX()-72)*-1;
        double calculatedPoseY = (follower.getPose().getY()-72)*1;
        Pose calculatedPose = new Pose(calculatedPoseY, calculatedPoseX, follower.getPose().getHeading()-Math.toRadians(90));
        //Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(calculatedPose, "#4CAF50");
        Drawing.sendPacket();
    }
    public void updateArmLimits() {
        // Extension limits
        if (robotCoreCustom.extHomingState != RobotCoreCustom.HomingState.HOMING) {
            if (targetArmPos[1] > robotCoreCustom.getMaxLength(targetArmPos[0])) {
                canExtend = false;
                targetArmPos[1] = robotCoreCustom.getMaxLength(targetArmPos[0]);
            } else canExtend = true;

            if (targetArmPos[1] < 50) {
                canRetract = false;
                targetArmPos[1] = 50;
            } else canRetract = true;

            robotCoreCustom.setArmExtPos(targetArmPos[1]);
        } else targetArmPos[1] = 0;

        // Rotation limits
        if (robotCoreCustom.rotHomingState != RobotCoreCustom.HomingState.HOMING) {
            if (targetArmPos[0] > 4000) {
                canTurnPositive = false;
                targetArmPos[0] = 4000;
            } else canTurnPositive = true;

            if (targetArmPos[0] < 20) {
                canTurnNegative = false;
                targetArmPos[0] = 20;
            } else canTurnNegative = true;

            if (targetSetPosTracking == MainDriveOpmode.targetSetPos.UP && upPosTracking == MainDriveOpmode.PosWait.GO || upPosTracking == MainDriveOpmode.PosWait.WAIT1 || upPosTracking == MainDriveOpmode.PosWait.WAIT) {
                robotCoreCustom.setArmRotPos(targetArmPos[0], 1);
            }
            if (targetSetPosTracking == MainDriveOpmode.targetSetPos.DOWN && downPosTracking == MainDriveOpmode.PosWait.GO || downPosTracking == MainDriveOpmode.PosWait.WAIT1 || downPosTracking == MainDriveOpmode.PosWait.WAIT) {
                robotCoreCustom.setArmRotPos(targetArmPos[0], 1);
            }
            if (targetSetPosTracking == MainDriveOpmode.targetSetPos.NA) {
                robotCoreCustom.setArmRotPos(targetArmPos[0], 1);
            }
            if (targetSetPosTracking == MainDriveOpmode.targetSetPos.VEL) {
                robotCoreCustom.setArmRotVel(targetArmVel[0]);
            }
        }
    }
    public void gamePadCont2DPos(Gamepad gamepad) {
        if (gamePadPollingRate.milliseconds() > pollingRateMS) {
            if (gamepad.left_stick_y < -0.2 || gamepad.left_stick_y > 0.2) {
                targetSetPosTracking = MainDriveOpmode.targetSetPos.VEL;
                robotCoreCustom.motorControllerRot.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else if(targetSetPosTracking == MainDriveOpmode.targetSetPos.VEL) {
                targetSetPosTracking = MainDriveOpmode.targetSetPos.NA;
                robotCoreCustom.motorControllerRot.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                targetArmPos[0] = robotCoreCustom.motorControllerRot.motor.getCurrentPosition();
            }
            targetArmVel[0] = (gamepad.left_stick_y * -1); // Rotation adjustment
            targetArmPos[1] += (gamepad.right_stick_y * -18); // Extension adjustment
        }
    }
}
