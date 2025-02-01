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

import drive.RobotCoreCustom;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name="TerraBiteDrive V1", group="!advanced")
public class MainDriveOpmode extends OpMode{
    RobotCoreCustom robotCoreCustom = new RobotCoreCustom();
    double[] targetArmPos = {0, 0}; // Arm position: {rotation, extension}
    double[] targetArmVel = {0, 0}; // Arm velocity: {rotation, extension}
    ElapsedTime gamePadPollingRate = new ElapsedTime();
    int pollingRateMS = 5; // Polling rate for gamepad input
    boolean canExtend, canRetract, canTurnPositive, canTurnNegative = true; // Arm limits
    Follower follower;
    Pose startPose = new Pose(7.5, 88.6, 0);
    ElapsedTime timer = new ElapsedTime();

    // Enums to track gripper states
    public enum gripperPos {
        OPEN,
        CLOSE
    }

    public enum gripperPitchPos {
        FORWARD,
        BACKWARD
    }

    public enum gripperRollerPos {
        FORWARD,
        STOPPED,
        REVERSE
    }
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
    double rotationalExponentialOutput;
    double rotationalSensitivity;


    // Gripper state tracking variables
    gripperRollerPos gripperRollerTracking;
    gripperPos gripperTracking;
    gripperPitchPos gripperPitchTracking;
    ElapsedTime gripperTimer;
    PosWait downPosTracking;
    PosWait upPosTracking;
    targetSetPos targetSetPosTracking = targetSetPos.NA;
    // Telemetry
    Telemetry telemetryA;


    @Override
    public void init() {
        // Initialize constants, follower, and robot hardware
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        robotCoreCustom.robotCoreInit(hardwareMap);
        gripperTimer = new ElapsedTime();
        gripperTimer.reset();
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
    }

    @Override
    public void init_loop() {
        // Optional: Logic for repeated initialization before start
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        timer.reset();
        gripperPitchTracking = gripperPitchPos.FORWARD;
        gripperRollerTracking = gripperRollerPos.STOPPED;
        gripperTracking = gripperPos.CLOSE;
        gripperTimer.reset();
        robotCoreCustom.homeRot();
    }

    @Override
    public void loop() {
        // Gamepad input handling
        gamePadCont2DPos(gamepad2);
        gripperChecking(gamepad2);

        // Homing functions for arm
        if (gamepad2.dpad_right) {
            robotCoreCustom.homeRot();
            targetArmPos[0] = 12;
            targetArmPos[1] = 50;
        }
        if (robotCoreCustom.rotHomingState == RobotCoreCustom.HomingState.SUCCESS) {
            robotCoreCustom.homeExt();
            targetArmPos[1] = 50;
            robotCoreCustom.rotHomingState = RobotCoreCustom.HomingState.IDLE;
        }
        if (robotCoreCustom.extHomingState == RobotCoreCustom.HomingState.SUCCESS) {
            robotCoreCustom.extHomingState = RobotCoreCustom.HomingState.IDLE;
        }
        if (gamepad2.right_stick_button) {
            robotCoreCustom.sweep();
        }
        // Set positions
        if (gamepad2.dpad_down) {
            targetSetPosTracking = targetSetPos.DOWN;
            downPosTracking = PosWait.GO;

        }
        if (downPosTracking == PosWait.WAIT && targetSetPosTracking == targetSetPos.DOWN) {
            targetArmPos[1] = (int) (850);
            //targetArmPos[0] = (int) (1200);
            if (robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() < 900) {
                downPosTracking = PosWait.GO;
            }
        }
        if (downPosTracking == PosWait.GO && targetSetPosTracking == targetSetPos.DOWN) {
            targetArmPos[0] = (int) (178);
            targetArmPos[1] = (int) (1800);
            targetSetPosTracking = targetSetPos.NA;
        }
        if (gamepad2.dpad_up) {
            targetSetPosTracking = targetSetPos.UP;
            upPosTracking = PosWait.GO;
        }

        if (upPosTracking == PosWait.WAIT && targetSetPosTracking == targetSetPos.UP) {
            targetArmPos[1] = (int) (450);
            if (robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() < 500) {
                upPosTracking = PosWait.WAIT1;
            }
        }
        if (upPosTracking == PosWait.WAIT1 && targetSetPosTracking == targetSetPos.UP) {
            targetArmPos[0] = (int) (633);
            if (robotCoreCustom.motorControllerRot.motor.getCurrentPosition() > 600) {
                upPosTracking = PosWait.GO;
            }
        }
        if (upPosTracking == PosWait.GO && targetSetPosTracking == targetSetPos.UP) {
            targetArmPos[0] = (int) (2450);
            targetArmPos[1] = (int) (4500);
            targetSetPosTracking = targetSetPos.NA;
        }
        if (robotCoreCustom.motorControllerExt1.motor.getCurrentPosition() > 4400) {
            gripperPitchTracking = gripperPitchPos.FORWARD;
        }

        // Update robot systems
        rotationalSensitivity = 0.5;  // Adjust this for desired sensitivity
        rotationalExponentialOutput = Math.signum(-gamepad1.right_stick_x) * Math.pow(Math.abs(gamepad1.right_stick_x), rotationalSensitivity);
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, (rotationalExponentialOutput), true);
        follower.update();
        updateArmLimits();
        robotCoreCustom.updateAll();

        // Telemetry for debugging
        telemetryA.addData("armState", targetSetPosTracking);
        telemetryA.addData("extTarget", robotCoreCustom.extTicks);
        telemetryA.addData("extPosition", robotCoreCustom.motorControllerExt0.motor.getCurrentPosition());
        telemetryA.addData("rotTarget", robotCoreCustom.motorControllerRot.motor.getTargetPosition());
        telemetryA.addData("rotPosition", robotCoreCustom.motorControllerRot.motor.getCurrentPosition());
        telemetryA.addData("rotPower", robotCoreCustom.motorControllerRot.motor.getPower());
        telemetryA.addData("targetPower", targetArmVel[0]);
        telemetryA.addData("poseX", follower.getPose().getX());
        telemetryA.addData("poseY", follower.getPose().getY());
        telemetryA.addData("poseHeading", follower.getPose().getHeading());
        telemetryA.addData("gripperPitch", robotCoreCustom.gripperPitch.getPosition());
        telemetryA.update();
    }

    // Handles gamepad input for 2D arm control
    public void gamePadCont2DPos(Gamepad gamepad) {
        if (gamePadPollingRate.milliseconds() > pollingRateMS) {
            if (gamepad.left_stick_y < -0.2 || gamepad.left_stick_y > 0.2) {
                targetSetPosTracking = targetSetPos.VEL;
                robotCoreCustom.motorControllerRot.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else if(targetSetPosTracking == targetSetPos.VEL) {
                targetSetPosTracking = targetSetPos.NA;
                robotCoreCustom.motorControllerRot.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                targetArmPos[0] = robotCoreCustom.motorControllerRot.motor.getCurrentPosition();
            }
            targetArmVel[0] = (gamepad.left_stick_y * -1); // Rotation adjustment
            targetArmPos[1] += (gamepad.right_stick_y * -30); // Extension adjustment
        }
    }

    // Checks and manages gripper controls
    public void gripperChecking(@NonNull Gamepad gamepad) {
        // Toggle roller states
        if ((gamepad.left_bumper) && gripperTimer.milliseconds() > 200) {
            gripperTimer.reset();
            switch (gripperRollerTracking) {
                case FORWARD:
                case REVERSE:
                    gripperRollerTracking = gripperRollerPos.STOPPED;
                    break;
                case STOPPED:
                    gripperRollerTracking = gripperRollerPos.FORWARD;
                    break;
            }
        }
            if ((gamepad.right_bumper) && gripperTimer.milliseconds() > 200) {
                gripperTimer.reset();
                switch (gripperRollerTracking) {
                    case REVERSE:
                    case FORWARD:
                        gripperRollerTracking = gripperRollerPos.STOPPED;
                        break;
                    case STOPPED:
                        gripperRollerTracking = gripperRollerPos.REVERSE;
                        break;
                }
            }

        // Toggle gripper open/close
        if (gamepad.a && gripperTimer.milliseconds() > 200) {
            gripperTimer.reset();
            gripperTracking = (gripperTracking == gripperPos.OPEN) ? gripperPos.CLOSE : gripperPos.OPEN;
        }

        // Toggle gripper pitch
        if (gamepad.b && gripperTimer.milliseconds() > 200) {
            gripperTimer.reset();
            gripperPitchTracking = (gripperPitchTracking == gripperPitchPos.FORWARD) ? gripperPitchPos.BACKWARD : gripperPitchPos.FORWARD;
            if (gripperRollerTracking != gripperRollerPos.STOPPED) {
                gripperRollerTracking = gripperRollerPos.STOPPED;
            }
        }

        // Update hardware states based on tracking
        if (gripperTracking == gripperPos.CLOSE) {
            gripperRollerTracking = gripperRollerPos.STOPPED;
        }
        robotCoreCustom.setGripperRollers(
                (gripperRollerTracking == gripperRollerPos.STOPPED) ? 0.5 : 1,
                (gripperRollerTracking == gripperRollerPos.REVERSE) ? RobotCoreCustom.Direction.REVERSE : RobotCoreCustom.Direction.FORWARD
        );

        robotCoreCustom.setGripperPitch(
                (gripperPitchTracking == gripperPitchPos.FORWARD) ? 0.6 : 0.15
                // Adjust pitch as needed
        );

        robotCoreCustom.setGripper(
                (gripperTracking == gripperPos.OPEN) ? 0.25 : 0.42 // Adjust positions for open/close
        );
    }

    // Updates arm limits based on position and homing state
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

            if (targetSetPosTracking == targetSetPos.UP && upPosTracking == PosWait.GO || upPosTracking == PosWait.WAIT1 || upPosTracking == PosWait.WAIT) {
                robotCoreCustom.setArmRotPos(targetArmPos[0], 1);
            }
            if (targetSetPosTracking == targetSetPos.DOWN && downPosTracking == PosWait.GO || downPosTracking == PosWait.WAIT1 || downPosTracking == PosWait.WAIT) {
                robotCoreCustom.setArmRotPos(targetArmPos[0], 1);
            }
            if (targetSetPosTracking == targetSetPos.NA) {
                robotCoreCustom.setArmRotPos(targetArmPos[0], 1);
            }
            if (targetSetPosTracking == targetSetPos.VEL) {
                robotCoreCustom.setArmRotVel(targetArmVel[0]);
            }
        }
    }
}
