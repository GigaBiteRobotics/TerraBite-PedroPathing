package drive.opmode;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import drive.RobotCoreCustom;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "Drive Beta", group = "!advanced")
public class MainDriveOpmode extends OpMode {
    RobotCoreCustom robotCoreCustom = new RobotCoreCustom();
    double[] diffPos = new double[]{0, 0};
    double extTargetPosition = 0;
    double wristPos = 0.5;
    Follower follower;
    Pose startPose = new Pose(0, 0, 0);
    ElapsedTime peckTimer = new ElapsedTime();
    ElapsedTime movementVectorTickTimer = new ElapsedTime();
    ElapsedTime robotStateDebounceTimer = new ElapsedTime();
    ElapsedTime gripperDebounceTimer = new ElapsedTime();
    int peckState = 0;

    enum RobotState {
        MANUAL,
        POSITION_MODE_1,
        POSITION_MODE_2
    }

    enum GripperState {
        OPEN,
        CLOSED
    }

    GripperState gripperState = GripperState.CLOSED;
    RobotState robotState = RobotState.MANUAL;

    @Override
    public void init() {
        robotCoreCustom.robotCoreInit(hardwareMap);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
    }

    @Override
    public void start() {
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
        robotCoreCustom.setExtPos(extTargetPosition);

        follower.update();
        telemetry.addData("Robot State", robotState);
        telemetry.addData("Extension Target Position", extTargetPosition);
        telemetry.addData("Extension Current Position", -robotCoreCustom.motorControllerExt0.motor.getCurrentPosition());
        telemetry.update();
    }

    public void updateGamepad2(@NonNull Gamepad gamepad) {
        if (robotState == RobotState.MANUAL) {
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
                    power = Math.max(0.2, Math.min(1, -gamepad.right_stick_y));
                }
                robotCoreCustom.motorControllerExt0.motor.setPower(power);
                robotCoreCustom.motorControllerExt1.motor.setPower(power);
                extTargetPosition = -robotCoreCustom.motorControllerExt0.motor.getCurrentPosition();
            } else {
                robotCoreCustom.enablePIDFExt = true;
            }
            if (gamepad.left_stick_y > 0.1 || gamepad.left_stick_y < -0.1) {
                wristPos = Math.max(0, Math.min(1, gamepad.left_stick_y + 0.5));
            }
        }

        if (robotState == RobotState.POSITION_MODE_1) {
            if (gamepad.left_stick_y > 0.2) {
                extTargetPosition = 485;
                robotCoreCustom.homeMotorRot(RobotCoreCustom.HomingState.UP);
            } else if (gamepad.left_stick_x < -0.2) {
                extTargetPosition = 200;
                robotCoreCustom.homeMotorRot(RobotCoreCustom.HomingState.UP);
            } else if (gamepad.left_stick_y < -0.2) {
                robotCoreCustom.homeMotorRot(RobotCoreCustom.HomingState.DOWN);
                robotCoreCustom.homeMotorExt();
            }
        }

        if (robotState == RobotState.POSITION_MODE_2) {
            if (gamepad.left_stick_y > 0.2) {
                extTargetPosition = 200;
                robotCoreCustom.homeMotorRot(RobotCoreCustom.HomingState.UP);
            } else if (gamepad.left_stick_x < -0.2) {
                extTargetPosition = 0;
                robotCoreCustom.homeMotorRot(RobotCoreCustom.HomingState.UP);
            } else if (gamepad.left_stick_y < -0.2) {
                robotCoreCustom.homeMotorRot(RobotCoreCustom.HomingState.DOWN);
                robotCoreCustom.homeMotorExt();
            }
        }

        diffPos[1] += gamepad.right_stick_x * -0.05;
        diffPos[0] = Math.max(-1, Math.min(1, diffPos[0]));
        diffPos[1] = Math.max(-0.28, Math.min(0.28, diffPos[1]));
        wristPos += gamepad.right_stick_x * 0.05;
        wristPos = Math.max(0, Math.min(1, wristPos));

        if (gamepad.right_trigger > 0.1) {
            if (peckState == 0) {
                peckStart();
            }
        } else {
            if (peckState != 0) {
                peckUpdate();
            }
        }

        if (gamepad.dpad_down && robotStateDebounceTimer.milliseconds() > 100) {
            if (robotState == RobotState.MANUAL || robotState == RobotState.POSITION_MODE_2) {
                robotState = RobotState.POSITION_MODE_1;
            } else if (robotState == RobotState.POSITION_MODE_1) {
                robotState = RobotState.MANUAL;
            }
            robotStateDebounceTimer.reset();
        } else if (gamepad.dpad_left && robotStateDebounceTimer.milliseconds() > 100) {
            if (robotState == RobotState.MANUAL || robotState == RobotState.POSITION_MODE_1) {
                robotState = RobotState.POSITION_MODE_2;
            } else {
                robotState = RobotState.MANUAL;
            }
            robotStateDebounceTimer.reset();
        }
        if (gamepad.left_bumper) {
            if (gamepad.a) {
                follower.setTeleOpMovementVectors(-1, 0, 0, true);
                movementVectorTickTimer.reset();
            } else if (gamepad.b) {
                follower.setTeleOpMovementVectors(1, 0, 0, true);
                movementVectorTickTimer.reset();
            } else if (gamepad.x) {
                follower.setTeleOpMovementVectors(0, -1, 0, true);
                movementVectorTickTimer.reset();
            } else if (gamepad.y) {
                follower.setTeleOpMovementVectors(0, 1, 0, true);
                movementVectorTickTimer.reset();
            } else {
                follower.setTeleOpMovementVectors(0, 0, 0, true); // Stop movement
            }
        }else {
            if (gamepad.a) {
                robotCoreCustom.homeMotorExt();
            }
            if (gamepad.b && gripperDebounceTimer.milliseconds() > 200) {
                if (gripperState == GripperState.CLOSED) {
                    robotCoreCustom.setGripper(0.35); // Open gripper
                } else {
                    robotCoreCustom.setGripper(0.57); // Close gripper
                }
            }
        }
    }

    public void updateGamepad1(@NonNull Gamepad gamepad) {
        follower.setTeleOpMovementVectors(-gamepad.left_stick_y, -gamepad.left_stick_x, gamepad.right_stick_x * -0.46, true);
    }

    public void peckStart() {
        peckState = 1;
        peckTimer.reset();
    }

    public void peckUpdate() {
        if (peckState == 1) {
            robotCoreCustom.setGripper(0.35);
            wristPos = 0.5;
            if (peckTimer.milliseconds() > 500) {
                peckState = 2;
            }
        } else if (peckState == 2) {
            robotCoreCustom.setGripper(0.57);
            wristPos = 0.5;
            if (peckTimer.milliseconds() > 1000) {
                peckState = 3;
            }
        } else if (peckState == 3) {
            robotCoreCustom.setGripper(0.35);
            wristPos = 0.5;
            if (peckTimer.milliseconds() > 1500) {
                peckState = 0;
            }
        }
    }
}