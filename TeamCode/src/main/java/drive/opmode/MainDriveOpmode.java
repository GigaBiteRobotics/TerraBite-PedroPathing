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
    double wristPos = 0.25;
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
        WALL_PICKUP, BUMP, PICKUP
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
        wristPos = 0.25;
        robotCoreCustom.homeMotorRot(RobotCoreCustom.HomingState.DOWN);
        gripperState = GripperState.CLOSED;
    }

    @Override
    public void loop() {
        updateGamepad2(gamepad2);
        updateGamepad1(gamepad1);

        robotCoreCustom.setDiffPos(diffPos);
        robotCoreCustom.setWrist(wristPos);
        robotCoreCustom.homingUpdate();
        if (robotCoreCustom.rotCurrentState == RobotCoreCustom.currentState.DOWN && extTargetPosition > 60) {
            extTargetPosition = 60;
        }
        if (gripperState == GripperState.CLOSED) {
            robotCoreCustom.setGripper(0.57); // Close gripper
        } else {
            robotCoreCustom.setGripper(0.35); // Open gripper
        }

        wristPos = Math.min(0.9, Math.max(0.25, wristPos));
        robotCoreCustom.setExtPos(extTargetPosition);
        updatePosition();

        follower.update();
        telemetry.addData("Robot State", robotState);
        telemetry.addData("Position State", positionState);
        telemetry.addData("Extension Target Position", extTargetPosition);
        telemetry.addData("Extension Current Position", -robotCoreCustom.motorControllerExt0.motor.getCurrentPosition());
        telemetry.addData("Extension Power", robotCoreCustom.motorControllerExt0.motor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Wrist Position", wristPos);
        telemetry.addData("Gripper State", gripperState);
        telemetry.addData("rotCurrentState", robotCoreCustom.rotCurrentState);
        telemetry.addData("diffPos", diffPos[0] + ", " + diffPos[1]);
        telemetry.addData("RealDiffPos", robotCoreCustom.servoDiffLeft.getPosition() + ", " + robotCoreCustom.servoDiffRight.getPosition());
        telemetry.update();
    }

    public void updateGamepad2(@NonNull Gamepad gamepad) {
        if (robotState == RobotState.MANUAL) {
            if (-gamepad.right_stick_y > 0.1 || -gamepad.right_stick_y < -0.1) {
                double power;
                robotCoreCustom.enablePIDFExt = false;
                if (robotCoreCustom.rotCurrentState == RobotCoreCustom.currentState.DOWN && -robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() < 60) {
                    power = Math.max(-1, Math.min(1, -gamepad.right_stick_y));
                } else if (robotCoreCustom.rotCurrentState == RobotCoreCustom.currentState.DOWN && -robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() > 60) {
                    power = Math.max(-1, Math.min(0, -gamepad.right_stick_y));
                } else if (-robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() > 650) {
                    power = Math.max(-1, Math.min(0.3, -gamepad.right_stick_y));
                } else if (-robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() < 5) {
                    power = Math.max(0, Math.min(1, -gamepad.right_stick_y));
                } else if (robotCoreCustom.rotCurrentState == RobotCoreCustom.currentState.UP && -robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() < 150) {
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
            if (gamepad.left_stick_y < -0.2) {
                positionState = PositionState.HIGH_SUB;
                positionStateIndex = 0;
            } else if (gamepad.left_stick_x < -0.2) {
                positionState = PositionState.LOW_SUB;
                positionStateIndex = 0;
            } else if (gamepad.left_stick_y > 0.2) {
                positionState = PositionState.WALL_PICKUP;
                positionStateIndex = 0;
            }
        }

        diffPos[1] += gamepad.right_stick_x * -0.05;
        diffPos[0] = Math.max(-2, Math.min(2, diffPos[0]));
        diffPos[1] = Math.max(-1, Math.min(1, diffPos[1]));
        //wristPos += gamepad.right_stick_x * 0.05;

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
                    robotCoreCustom.setGripper(0.57);// Close gripper
                    gripperState = GripperState.CLOSED;
                }
                gripperDebounceTimer.reset();
            }
        }
        if (gamepad.left_trigger > 0.1 && robotCoreCustom.rotCurrentState == RobotCoreCustom.currentState.UP) {
            positionState = PositionState.BUMP;
            positionStateIndex = 0;
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
            gripperState = GripperState.OPEN; // Open gripper
            wristPos = 0.76;
            peckState = 2;
            diffPos[0] = 1;
            peckTimer.reset();
        } else if (peckState == 2) {
            if (peckTimer.milliseconds() > 50) {
                wristPos = 0.83;
                peckState = 3;
                peckTimer.reset();
            }
        } else if (peckState == 3 && peckTimer.milliseconds() > 100) {
            gripperState = GripperState.CLOSED;
            peckState = 4;
        } else if (peckState == 4 && peckTimer.milliseconds() > 50) {
            wristPos = 0.76;
            peckState = 0;
        }
    }
    public void updatePosition() {
        if (positionState == PositionState.HIGH_BASKET) {
            if (positionStateIndex == 0) {
                extTargetPosition = 0;
                robotCoreCustom.homeMotorRot(RobotCoreCustom.HomingState.UP);
                positionStateIndex = 1;
                setPositionTimer.reset();
                diffPos = new double[]{0, 0};
            } else if (positionStateIndex == 1 && robotCoreCustom.rotCurrentState == RobotCoreCustom.currentState.UP) {
                robotCoreCustom.homeMotorExt();
                extTargetPosition = 600;
                positionStateIndex = 2;
            } else if (positionStateIndex == 2 && -robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() > 550) {
                wristPos = 0.65;
                positionStateIndex = 0;
                positionState = PositionState.IDLE;
            }
        }
        if (positionState == PositionState.LOW_BASKET) {
            if (positionStateIndex == 0) {
                extTargetPosition = 0;
                robotCoreCustom.homeMotorRot(RobotCoreCustom.HomingState.UP);
                positionStateIndex = 1;
                setPositionTimer.reset();
                diffPos = new double[]{0, 0};
            } else if (positionStateIndex == 1 && robotCoreCustom.rotCurrentState == RobotCoreCustom.currentState.UP) {
                robotCoreCustom.homeMotorExt();
                extTargetPosition = 100;
                positionStateIndex = 2;
            } else if (positionStateIndex == 2 && -robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() > 80) {
                wristPos = 0.65;
                positionStateIndex = 0;
                positionState = PositionState.IDLE;
            }
        }
        if (positionState == PositionState.HIGH_SUB) {
            if (positionStateIndex == 0) {
                robotCoreCustom.homeMotorExt();
                wristPos = 0.44;
                positionStateIndex = 1;
                diffPos = new double[]{1, 0};
                setPositionTimer.reset();
            } else if (positionStateIndex == 1 && setPositionTimer.milliseconds() > 500) {
                robotCoreCustom.homeMotorRot(RobotCoreCustom.HomingState.UP);
                positionStateIndex = 2;
            } else if (positionStateIndex == 2 && robotCoreCustom.rotCurrentState == RobotCoreCustom.currentState.UP) {
                extTargetPosition = 270;
                positionState = PositionState.IDLE;
            }
        }
        if (positionState == PositionState.PICKUP) {
            if (positionStateIndex == 0) {
                gripperState = GripperState.OPEN;
                extTargetPosition = 0;
                positionStateIndex = 1;
                setPositionTimer.reset();
                wristPos = 0.76;
                robotCoreCustom.homeMotorExt();
            } else if (positionStateIndex == 1 && -robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() < 25) {
                robotCoreCustom.homeMotorRot(RobotCoreCustom.HomingState.DOWN);
                extTargetPosition = 60;
                diffPos = new double[]{1, 0};
                positionStateIndex = 0;
                positionState = PositionState.IDLE;
            }
        }
        if (positionState == PositionState.WALL_PICKUP) {
            if (positionStateIndex == 0) {
                extTargetPosition = 0;
                positionStateIndex = 1;
                gripperState = GripperState.OPEN;
                wristPos = 0.68;
                diffPos = new double[]{0, 0};
                setPositionTimer.reset();
        } else if (positionStateIndex == 1 && -robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() < 25) {
                robotCoreCustom.homeMotorExt();
                robotCoreCustom.homeMotorRot(RobotCoreCustom.HomingState.DOWN);
                extTargetPosition = 0;
                wristPos = 0.68;
                positionStateIndex = 0;
                positionState = PositionState.IDLE;
            }
        }
        if (positionState == PositionState.BUMP) {
            if (positionStateIndex == 0) {
                extTargetPosition += 50;
                setPositionTimer.reset();
                positionStateIndex = 1;
            } else if (positionStateIndex == 1 && setPositionTimer.milliseconds() > 200) {
                gripperState = GripperState.OPEN;
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
     Right bumper = peck DONE
     right stick x(horizontal) = wrist rotation     DONE
     right stick y(vertical) = extension        DONE
     Dpad down = toggle between position mode 1 and manual mode  DONE
     Dpad left = toggle between position mode 2 and manual mode     DONE
     manual mode = manual control of extension and wrist        DONE
        position mode 1:
        left stick up is set pos for high bucket        DONE
        left stick left is set pos for low bucket   DONE
        left stick down is set pos for pickup  DONE

        position mode 2:
        left stick up is set pos for high sub hang
        left stick left is set pos for low sub hang
        left stick down is set pos for wall pickup DONE

    when holding left bumper a,b,x,y will move the robot follower slightly in the direction of the button pressed
    otherwise a,b,x,y will control gripper etc.  DONE
 */