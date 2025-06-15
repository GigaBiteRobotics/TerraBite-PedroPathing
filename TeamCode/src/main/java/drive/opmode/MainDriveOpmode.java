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
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

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

    }

    @Override
    public void loop() {
        updateGamepad2(gamepad2);
        robotCoreCustom.setDiffPos(diffPos);
        robotCoreCustom.homingUpdate();
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x * -0.46, true);
        if (gamepad2.left_stick_y > 0.5) {
            robotCoreCustom.homeMotorRot(RobotCoreCustom.HomingState.DOWN);
        }
        if (gamepad2.left_stick_y < -0.5) {
            robotCoreCustom.homeMotorRot(RobotCoreCustom.HomingState.UP);
        }
        if (gamepad2.a) {
            robotCoreCustom.homeMotorExt();
        }
        follower.update();
        telemetry.addData("extHomingState", robotCoreCustom.extHomingState);
        telemetry.addData("rotHomingState", robotCoreCustom.rotHomingState);
        telemetry.addData("Ext AMPS", robotCoreCustom.motorControllerExt0.motor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Rot AMPS", robotCoreCustom.motorControllerRot0.motor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Diff Pos", "X: %.2f, Y: %.2f", diffPos[0], diffPos[1]);
        telemetry.addData("Ext Target Position", extTargetPosition);
        telemetry.update();
    }

    // In MainDriveOpmode.java, updateGamepad:
    // In updateGamepad:
    public void updateGamepad2(@NonNull Gamepad gamepad) {
        //diffPos[0] += 0.05 * gamepad.left_stick_y;

        // Rotate: right bumper increases, left bumper decreases
        if (gamepad.right_bumper) {
            diffPos[1] += 0.05;
        }
        if (gamepad.left_bumper) {
            diffPos[1] -= 0.05;
        }

        diffPos[0] = Math.max(-1, Math.min(1, diffPos[0]));
        diffPos[1] = Math.max(-1, Math.min(1, diffPos[1]));

        extTargetPosition += gamepad2.right_stick_y * -2; // Adjust the scaling factor as needed

        robotCoreCustom.setExtPos(extTargetPosition); // Adjust the scaling factor as needed
    }
}