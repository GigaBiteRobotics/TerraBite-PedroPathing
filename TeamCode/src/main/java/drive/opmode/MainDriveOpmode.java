package drive.opmode;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import drive.RobotCoreCustom;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name="Drive Beta", group="!advanced")
public class MainDriveOpmode extends OpMode {
    RobotCoreCustom robotCoreCustom = new RobotCoreCustom();
    double[] diffPos = new double[]{0, 0}; // Initialize with zero values
    double extTargetPosition = 0;
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
        robotCoreCustom.homeMotorExt();
        robotCoreCustom.homeMotorRot(RobotCoreCustom.HomingState.UP);
    }

    @Override
    public void loop() {
        updateGamepad2(gamepad2);
        updateGamepad1(gamepad1);

        robotCoreCustom.setDiffPos(diffPos);
        robotCoreCustom.homingUpdate();
        //follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x * -0.46, true);
        if (gamepad2.left_stick_y > 0.5) {
            robotCoreCustom.homeMotorRot(RobotCoreCustom.HomingState.DOWN);
        }
        if (gamepad2.left_stick_y < -0.5) {
            robotCoreCustom.homeMotorRot(RobotCoreCustom.HomingState.UP);
        }
        if (gamepad2.a) {
            robotCoreCustom.homeMotorExt();
            extTargetPosition = 0; // Reset target position when homing
        }
        if (gamepad2.b) {
            extTargetPosition = 600; // Set target position to 300 when button B is pressed
        }
        if (gamepad2.x) {
            extTargetPosition = 0; // Set target position to 0 when button X is pressed
        }
        robotCoreCustom.setExtPos(extTargetPosition);

        // Telemetry data
        follower.update();
        telemetry.addData("extHomingState", robotCoreCustom.extHomingState);
        telemetry.addData("rotHomingState", robotCoreCustom.rotHomingState);
        telemetry.addData("Ext AMPS", robotCoreCustom.motorControllerExt0.motor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Ext Power", robotCoreCustom.motorControllerExt0.motor.getPower());
        telemetry.addData("Rot AMPS", robotCoreCustom.motorControllerRot0.motor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Rot Power", robotCoreCustom.motorControllerRot0.motor.getPower());
        telemetry.addData("Diff Pos", "X: %.2f, Y: %.2f", diffPos[0], diffPos[1]);
        telemetry.addData("Ext Target Position", extTargetPosition);
        telemetry.addData("Ext Position", robotCoreCustom.motorControllerExt0.motor.getCurrentPosition());
        telemetry.addData("RealDiffPos", "X: %.2f, Y: %.2f", robotCoreCustom.servoDiffLeft.getPosition(), robotCoreCustom.servoDiffRight.getPosition());
        telemetry.update();
    }

    // In MainDriveOpmode.java, updateGamepad:
    // In updateGamepad:
    public void updateGamepad2(@NonNull Gamepad gamepad) {
        extTargetPosition += gamepad2.right_stick_y * -12; // Adjust the scaling factor as needed
    }
    public void updateGamepad1(@NonNull Gamepad gamepad) {
        // Update the diffPos array based on gamepad input

        diffPos[0] += gamepad1.left_stick_y * 0.1; // Adjust the scaling factor as needed
        diffPos[1] += gamepad1.left_stick_x * -0.05; // Adjust the scaling factor as needed

        // Ensure diffPos values are within a reasonable range
        diffPos[0] = Math.max(-1, Math.min(1, diffPos[0]));
        diffPos[1] = Math.max(-0.28, Math.min(0.28, diffPos[1]));
    }
}