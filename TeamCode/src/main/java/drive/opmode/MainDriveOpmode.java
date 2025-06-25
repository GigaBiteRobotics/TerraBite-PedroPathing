package drive.opmode;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import drive.RobotCoreCustom;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "Drive", group = "!advanced")
public class MainDriveOpmode extends OpMode {
    RobotCoreCustom robotCoreCustom = new RobotCoreCustom();
    double[] diffPos = new double[]{0, 0};
    double extTargetPosition = 0;
    double wristPos = 0;
    Follower follower;
    Pose startPose = new Pose(0, 0, 0);
    ElapsedTime peckTimer = new ElapsedTime();
    ElapsedTime movementVectorTickTimer = new ElapsedTime();
    ElapsedTime robotStateDebounceTimer = new ElapsedTime();
    ElapsedTime gripperDebounceTimer = new ElapsedTime();
    ElapsedTime setPositionTimer = new ElapsedTime();
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
    enum PositionState {
        HIGH_BASKET,
        LOW_BASKET,
        HIGH_SUB,
        LOW_SUB,
        IDLE,
        PICKUP
    }
    int positionStateIndex = 0;

    GripperState gripperState = GripperState.CLOSED;
    RobotState robotState = RobotState.MANUAL;

    PositionState positionState = PositionState.IDLE;

    @Override
    public void init() {
        robotCoreCustom.robotCoreInit(hardwareMap);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
    }

    @Override
    public void start() {
        setPositionTimer.reset();
        follower.startTeleopDrive();
        robotCoreCustom.homeMotorExt();
        extTargetPosition = 0;
        wristPos = 0;
        robotCoreCustom.homeMotorRot(RobotCoreCustom.HomingState.DOWN);
        robotCoreCustom.setGripper(0.57);
    }

    @Override
    public void loop() {
        updateGamepad2(gamepad2);
        updateGamepad1(gamepad1);

        robotCoreCustom.setDiffPos(diffPos);
        robotCoreCustom.setWrist(wristPos);
        robotCoreCustom.homingUpdate();
        if (robotCoreCustom.rotCurrentState == RobotCoreCustom.currentState.DOWN && extTargetPosition > 85) {
            extTargetPosition = 85;
        }
        robotCoreCustom.setExtPos(extTargetPosition);
        updatePosition();

        follower.update();
        telemetry.addData("Robot State", robotState);
        telemetry.addData("Position State", positionState);
        telemetry.addData("Extension Target Position", extTargetPosition);
        telemetry.addData("Extension Current Position", -robotCoreCustom.motorControllerExt0.motor.getCurrentPosition());
        telemetry.addData("Wrist Position", wristPos);
        telemetry.addData("Gripper State", gripperState);
        telemetry.addData("rotCurrentState", robotCoreCustom.rotCurrentState);
        telemetry.addData("rotPosition0", robotCoreCustom.motorControllerRot0.motor.getCurrentPosition());
        telemetry.addData("right Trigger", gamepad2.right_trigger);
        telemetry.update();
    }

    public void updateGamepad2(@NonNull Gamepad gamepad) {
        if (robotState == RobotState.MANUAL) {
            if (-gamepad.right_stick_y > 0.1 || -gamepad.right_stick_y < -0.1) {
                double power;
                robotCoreCustom.enablePIDFExt = false;
                if (robotCoreCustom.rotCurrentState == RobotCoreCustom.currentState.DOWN && -robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() < 85) {
                    power = Math.max(-1, Math.min(1, -gamepad.right_stick_y));
                } else if (robotCoreCustom.rotCurrentState == RobotCoreCustom.currentState.DOWN && -robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() > 85) {
                    power = Math.max(-1, Math.min(0, -gamepad.right_stick_y));
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
                wristPos += gamepad.left_stick_y * 0.05;
                wristPos = Math.max(0, Math.min(1, wristPos));
            }
            if (gamepad.y && !gamepad.left_bumper) {
                robotCoreCustom.homeMotorRot(RobotCoreCustom.HomingState.UP);
            } else if (gamepad.x && !gamepad.left_bumper) {
                robotCoreCustom.homeMotorRot(RobotCoreCustom.HomingState.DOWN);
            }
        }

        if (robotState == RobotState.POSITION_MODE_1) {
            if (gamepad.left_stick_y < -0.2) {
                positionState = PositionState.HIGH_BASKET;
                positionStateIndex = 0;
            } else if (gamepad.left_stick_x < -0.2) {
                positionState = PositionState.LOW_BASKET;
                positionStateIndex = 0;
            } else if (gamepad.left_stick_y > 0.2) {
                positionState = PositionState.PICKUP;
                positionStateIndex = 0;
            }
        }

        if (robotState == RobotState.POSITION_MODE_2) {
            if (gamepad.left_stick_y > 0.2) {
                positionState = PositionState.HIGH_SUB;
                positionStateIndex = 0;
            } else if (gamepad.left_stick_x < -0.2) {
                positionState = PositionState.LOW_SUB;
                positionStateIndex = 0;
            } else if (gamepad.left_stick_y > 0.2) {
                positionState = PositionState.PICKUP;
                positionStateIndex = 0;
            }
        }

        diffPos[1] += gamepad.right_stick_x * -0.05;
        diffPos[0] = Math.max(-1, Math.min(1, diffPos[0]));
        diffPos[1] = Math.max(-0.28, Math.min(0.28, diffPos[1]));
        //wristPos += gamepad.right_stick_x * 0.05;
        wristPos = Math.max(0, Math.min(1, wristPos));

        if (gamepad.right_bumper) {
            if (peckState == 0) {
                peckStart();
            }
        } else {
            if (peckState != 0) {
                peckUpdate();
            }
        }

        if (gamepad.dpad_down && robotStateDebounceTimer.milliseconds() > 200) {
            if (robotState == RobotState.MANUAL || robotState == RobotState.POSITION_MODE_2) {
                robotState = RobotState.POSITION_MODE_1;
            } else if (robotState == RobotState.POSITION_MODE_1) {
                robotState = RobotState.MANUAL;
            }
            robotStateDebounceTimer.reset();
        } else if (gamepad.dpad_left && robotStateDebounceTimer.milliseconds() > 200) {
            if (robotState == RobotState.MANUAL || robotState == RobotState.POSITION_MODE_1) {
                robotState = RobotState.POSITION_MODE_2;
            } else {
                robotState = RobotState.MANUAL;
            }
            robotStateDebounceTimer.reset();
        }
        if (gamepad.left_bumper) {
            if (gamepad.a) {
                follower.setTeleOpMovementVectors(-0.4, 0, 0, true);
                movementVectorTickTimer.reset();
            } else if (gamepad.y) {
                follower.setTeleOpMovementVectors(0.4, 0, 0, true);
                movementVectorTickTimer.reset();
            } else if (gamepad.x) {
                follower.setTeleOpMovementVectors(0, 0.4, 0, true);
                movementVectorTickTimer.reset();
            } else if (gamepad.b) {
                follower.setTeleOpMovementVectors(0, -0.4, 0, true);
                movementVectorTickTimer.reset();
            } else {
                follower.setTeleOpMovementVectors(0, 0, 0, true); // Stop movement
            }
        }else {
            if (gamepad.a && !(positionState == PositionState.HIGH_SUB || positionState == PositionState.LOW_SUB)) {
                robotCoreCustom.homeMotorExt();
                extTargetPosition = 0;
            }
            if (gamepad.b && gripperDebounceTimer.milliseconds() > 200) {
                if (gripperState == GripperState.CLOSED) {
                    robotCoreCustom.setGripper(0.35); // Open gripper
                    gripperState = GripperState.OPEN;
                } else {
                    robotCoreCustom.setGripper(0.57); // Close gripper
                    gripperState = GripperState.CLOSED;
                }
                gripperDebounceTimer.reset();
            }
        }
    }

    public void updateGamepad1(@NonNull Gamepad gamepad) {
        if (!gamepad2.left_bumper) {
            follower.setTeleOpMovementVectors(-gamepad.left_stick_y, -gamepad.left_stick_x, gamepad.right_stick_x * -0.67, true);
        }
    }

    public void peckStart() {
        peckState = 1;
        peckTimer.reset();
    }

    public void peckUpdate() {
        if (peckState == 1) {
            robotCoreCustom.setGripper(0.35);
            wristPos = 0.9;
            if (peckTimer.milliseconds() > 500) {
                peckState = 2;
            }
        } else if (peckState == 2) {
            robotCoreCustom.setGripper(0.57);
            wristPos = 0.9;
            if (peckTimer.milliseconds() > 1000) {
                peckState = 3;
            }
        } else if (peckState == 3) {
            robotCoreCustom.setGripper(0.35);
            wristPos = 0.9;
            if (peckTimer.milliseconds() > 1500) {
                peckState = 0;
            }
        }
    }
    public void updatePosition() {
        if (positionState == PositionState.HIGH_BASKET) {
            if (positionStateIndex == 0) {
                robotCoreCustom.homeMotorExt();
                robotCoreCustom.homeMotorRot(RobotCoreCustom.HomingState.UP);
                positionStateIndex = 1;
                setPositionTimer.reset();
            } else if (positionStateIndex == 1 && robotCoreCustom.rotCurrentState == RobotCoreCustom.currentState.UP) {
                extTargetPosition = 485;
                wristPos = 0.5;
                positionStateIndex = 0;
                positionState = PositionState.IDLE;
            }
        }
        if (positionState == PositionState.LOW_BASKET) {
            if (positionStateIndex == 0) {
                robotCoreCustom.homeMotorRot(RobotCoreCustom.HomingState.UP);
                positionStateIndex = 1;
                setPositionTimer.reset();
            } else if (positionStateIndex == 1 && setPositionTimer.milliseconds() > 500) {
                extTargetPosition = 200;
                wristPos = 0.5;
                positionStateIndex = 0;
                positionState = PositionState.IDLE;
            }
        }
        if (positionState == PositionState.HIGH_SUB) {
            if (positionStateIndex == 0) {
                robotCoreCustom.homeMotorExt();
                robotCoreCustom.homeMotorRot(RobotCoreCustom.HomingState.DOWN);
                positionStateIndex = 1;
                setPositionTimer.reset();
            } else if (positionStateIndex == 1 && robotCoreCustom.rotCurrentState == RobotCoreCustom.currentState.UP && gamepad2.a) {
                extTargetPosition = 150;
                wristPos = 0.5;
                positionStateIndex = 0;
                positionState = PositionState.IDLE;
            }
        }
        if (positionState == PositionState.PICKUP) {
            if (positionStateIndex == 0) {
                robotCoreCustom.homeMotorExt();
                positionStateIndex = 1;
                setPositionTimer.reset();
            } else if (positionStateIndex == 1 && -robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() < 25) {
                robotCoreCustom.homeMotorRot(RobotCoreCustom.HomingState.DOWN);
                extTargetPosition = 85;
                diffPos = new double[]{0, 0};
                wristPos = 0.8;
                positionStateIndex = 0;
                positionState = PositionState.IDLE;
            }
        }
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
     Dpad down = toggle between position mode 1 and manual mode  DONE
     Dpad left = toggle between position mode 2 and manual mode     DONW
     manual mode = manual control of extension and wrist        DONE
        position mode 1:
        left stick up is set pos for high bucket        DONE
        left stick left is set pos for low bucket
        left stick down is set pos for pickup

        position mode 2:
        left stick up is set pos for high sub hang      DONE
        left stick left is set pos for low sub hang
        left stick down is set pos for pickup

    when holding left bumper a,b,x,y will move the robot follower slightly in the direction of the button pressed
    otherwise a,b,x,y will control gripper etc.
 */