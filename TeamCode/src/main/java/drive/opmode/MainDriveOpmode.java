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
    double wristPos = 0.5;
    Follower follower;
    Pose startPose = new Pose(0, 0, 0);

    public enum TeleState {

    }

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
        robotCoreCustom.setWrist(wristPos);
        robotCoreCustom.homingUpdate();
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x * -0.46, true);
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
        telemetry.addData("Wrist Position", wristPos);
        telemetry.update();
    }

    public void updateGamepad2(@NonNull Gamepad gamepad) {
        if (-gamepad.right_stick_y > 0.1 || -gamepad.right_stick_y < -0.1) {
            double power;
            robotCoreCustom.enablePIDFExt = false;
            if (robotCoreCustom.rotCurrentState == RobotCoreCustom.currentState.DOWN) {
                power = Math.max(-1, Math.min(1, -gamepad.right_stick_y));
            } else if (-robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() > 490) {
                power = Math.max(-1, Math.min(0.3, -gamepad.right_stick_y));
            } else if (-robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() < 5) {
                power = Math.max(0, Math.min(1, -gamepad.right_stick_y));
            } else if (robotCoreCustom.rotCurrentState == RobotCoreCustom.currentState.UP && -robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() < 300) {
                power = Math.max(-0.2, Math.min(1, -gamepad.right_stick_y));
            } else {
                power = Math.max(-1, Math.min(1, -gamepad.right_stick_y));
            }
            if (robotCoreCustom.motorControllerExt0.motor.getVelocity() > 400 && -robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() < 50) {
                power = Math.max(0.2, Math.min(1, -gamepad.right_stick_y)); // Prevents slamming when the extension is at the bottom
            }
            robotCoreCustom.motorControllerExt0.motor.setPower(power);
            robotCoreCustom.motorControllerExt1.motor.setPower(power);
            extTargetPosition = -robotCoreCustom.motorControllerExt0.motor.getCurrentPosition();
        } else {robotCoreCustom.enablePIDFExt = true;}

        if (gamepad.y) {
            robotCoreCustom.homeMotorExt();
        }


        //diffPos[0] += gamepad.right_stick_y * 0.1; // Adjust the scaling factor as needed
        diffPos[1] += gamepad.right_stick_x * -0.05; // Adjust the scaling factor as needed

        // Ensure diffPos values are within a reasonable range
        diffPos[0] = Math.max(-1, Math.min(1, diffPos[0]));
        diffPos[1] = Math.max(-0.28, Math.min(0.28, diffPos[1]));

        // Update wristPos based on gamepad input
        wristPos += gamepad1.right_stick_x * 0.05; // Adjust the scaling factor as needed
        // Ensure wristPos is within a reasonable range
        wristPos = Math.max(0, Math.min(1, wristPos));
    }
    public void updateGamepad1(@NonNull Gamepad gamepad) {
        // Update the diffPos array based on gamepad input


    }
}


/*
     High Bucket Ext = 485
     Absolute Max Ext = 490


     Keybinds:
     Connor's Keybinds:
     Right trigger = peck
     right stick x(horizontal) = wrist rotation     DONE
     right stick y(vertical) = extension        DONE
     Dpad down = toggle between position mode 1 and manual mode
     Dpad left = toggle between position mode 2 and manual mode
     manual mode = manual control of extension and wrist
        position mode 1:
        left stick up is set pos for high bucket
        left stick left is set pos for low bucket
        left stick down is set pos for pickup

        position mode 2:
        left stick up is set pos for high sub hang
        left stick left is set pos for low sub hang
        left stick down is set pos for pickup

    when holding left bumper a,b,x,y will move the robot follower slightly in the direction of the button pressed
    otherwise a,b,x,y will control gripper etc.
 */