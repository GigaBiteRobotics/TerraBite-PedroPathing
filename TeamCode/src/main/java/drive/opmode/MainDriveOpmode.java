package drive.opmode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import drive.CameraCoreCustom;
import drive.RobotCoreCustom;
import drive.RobotCoreCustomOLD;
import drive.RotationDataProcessorCustom;
import gigabite.robot.Robot;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name="Drive Beta", group="!advanced")
public class MainDriveOpmode extends OpMode {
    RobotCoreCustom robotCoreCustom = new RobotCoreCustom();
    double[] diffPos = new double[]{0, 0}; // Initialize with zero values
    Follower follower;
    Pose startPose = new Pose(0, 0, 0);

    @Override
    public void init() {
        robotCoreCustom.robotCoreInit(hardwareMap);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
    }

    @Override
    public void start(){
        follower.startTeleopDrive();

    }

    @Override
    public void loop() {
        updateGamepad(gamepad2);
        robotCoreCustom.setDiffPos(diffPos);
        robotCoreCustom.updateAll();
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x * -0.46, true);
        follower.update();

        telemetry.addData("Diff Pos", "X: %.2f, Y: %.2f", diffPos[0], diffPos[1]);
        telemetry.update();
    }

    // In MainDriveOpmode.java, updateGamepad:
    // In updateGamepad:
    public void updateGamepad(@NonNull Gamepad gamepad) {
        // Tilt: up/down on left stick
        diffPos[0] += 0.05 * gamepad.left_stick_y;
        // Rotate: left/right on right stick
        diffPos[1] += 0.05 * gamepad.right_stick_x;
        diffPos[0] = Math.max(-1, Math.min(1, diffPos[0]));
        diffPos[1] = Math.max(-1, Math.min(1, diffPos[1]));
    }
}