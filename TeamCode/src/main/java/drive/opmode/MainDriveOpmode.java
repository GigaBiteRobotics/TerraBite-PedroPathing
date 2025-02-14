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
import drive.RotationDataProcessorCustom;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name="TerraBiteDrive V1", group="!advanced")
public class MainDriveOpmode extends OpMode {
    // Initialize robot components and variables
    RobotCoreCustom robotCoreCustom = new RobotCoreCustom();
    RotationDataProcessorCustom rotationDataProcessor = new RotationDataProcessorCustom();
    CameraCoreCustom cameraCoreCustom = new CameraCoreCustom();

    // Arm position and velocity targets
    double[] targetArmPos = {0, 0}; // Arm position: {rotation, extension}
    double[] targetArmVel = {0, 0}; // Arm velocity: {rotation, extension}

    // Timers for various tasks
    ElapsedTime gamePadPollingRate = new ElapsedTime();
    int pollingRateMS = 5; // Polling rate for gamepad input in milliseconds
    boolean canExtend, canRetract, canTurnPositive, canTurnNegative = true; // Arm movement limits
    Follower follower;
    Pose startPose = new Pose(7.5, 88.6, 0); // Starting position of the robot
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime grabSampleTimer = new ElapsedTime();
    ElapsedTime cameraButtonTimer = new ElapsedTime();
    ElapsedTime cameraUpdateTimer = new ElapsedTime();
    ElapsedTime cameraResetTimer = new ElapsedTime();
    double cameraResetTimerMS = 1000; // Time to reset camera state
    boolean cameraWasFound = false; // Tracks if the camera has found a target
    int cameraFrameCount = 0; // Counts frames processed by the camera

    // Enums to track various states
    public enum gripperPos { OPEN, CLOSE } // Gripper open/close states
    public enum gripperPitchPos { FORWARD, BACKWARD, MIDDLE } // Gripper pitch positions
    public enum PosWait { WAIT, WAIT1, GO } // Position waiting states
    public enum targetSetPos { DOWN, UP, NA, VEL } // Arm target positions
    public enum gripperRotationState { AUTO, MANUAL } // Gripper rotation control modes
    public enum cameraState { SEARCHING, AVERAGING, FOUND, IDLE } // Camera states
    public enum cameraColor { RED, BLUE, YELLOW } // Camera color pipelines

    // State tracking variables
    cameraState cameraStateTracking = cameraState.SEARCHING;
    cameraColor cameraColorTracking = cameraColor.YELLOW;
    gripperRotationState gripperRotationStateTracking = gripperRotationState.MANUAL;
    double rotationalExponentialOutput; // Exponential output for rotational control
    double rotationalSensitivity; // Sensitivity for rotational control
    double rotationalForwardOffset; // Forward offset for rotational control
    int gripperGrabSamplePos = 0; // Gripper sample grabbing position

    // Gripper state tracking variables
    gripperPos gripperTracking;
    gripperPitchPos gripperPitchTracking;
    ElapsedTime gripperTimer;
    PosWait downPosTracking;
    PosWait upPosTracking;
    targetSetPos targetSetPosTracking = targetSetPos.NA;

    // Telemetry for debugging
    Telemetry telemetryA;
    double gripperRotationPosTarget = 0.474; // Target position for gripper rotation

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
        cameraCoreCustom.limelightCoreCustomInit(hardwareMap); // Initialize camera
    }

    @Override
    public void init_loop() {
        // Optional: Logic for repeated initialization before start
    }

    @Override
    public void start() {
        // Start teleop drive and reset timers
        follower.startTeleopDrive();
        timer.reset();
        gripperPitchTracking = gripperPitchPos.BACKWARD;
        gripperTracking = gripperPos.CLOSE;
        gripperTimer.reset();
        robotCoreCustom.homeRot(); // Home the rotation arm
    }

    @Override
    public void loop() {
        // Gamepad input handling
        gamePadCont2DPos(gamepad2);
        gripperChecking(gamepad2);
        cameraButtonUpdate();

        try {
            cameraRotationUpdate();
        } catch (Exception ignored) {}

        // Homing functions for arm
        if (gamepad2.dpad_right) {
            robotCoreCustom.homeRot();
        }
        if (robotCoreCustom.rotHomingState == RobotCoreCustom.HomingState.SUCCESS) {
            robotCoreCustom.homeExt();
            targetArmPos[1] = 50;
            robotCoreCustom.rotHomingState = RobotCoreCustom.HomingState.IDLE;
        }
        if (robotCoreCustom.extHomingState == RobotCoreCustom.HomingState.SUCCESS) {
            gripperPitchTracking = gripperPitchPos.FORWARD;
            robotCoreCustom.extHomingState = RobotCoreCustom.HomingState.IDLE;
            targetArmPos[0] = 240;
            targetArmPos[1] = 250;
        }
        if (gamepad2.right_stick_button) {
            robotCoreCustom.sweep();
        }

        // Set positions
        if (gamepad2.dpad_down) {
            targetSetPosTracking = targetSetPos.DOWN;
            downPosTracking = PosWait.WAIT;
            gripperPitchTracking = gripperPitchPos.FORWARD;
        }
        if (downPosTracking == PosWait.WAIT && targetSetPosTracking == targetSetPos.DOWN) {
            targetArmPos[1] = (int) (100);
            if (robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() < 1600) {
                downPosTracking = PosWait.GO;
            }
        }
        if (downPosTracking == PosWait.GO && targetSetPosTracking == targetSetPos.DOWN) {
            gripperPitchTracking = gripperPitchPos.MIDDLE;
            targetArmPos[0] = (int) (245);
            targetArmPos[1] = (int) (1800);
            targetSetPosTracking = targetSetPos.NA;
        }

        if (gamepad2.dpad_up) {
            targetSetPosTracking = targetSetPos.UP;
            gripperRotationPosTarget = 0.474;
            upPosTracking = PosWait.WAIT;
        }
        if (upPosTracking == PosWait.WAIT && targetSetPosTracking == targetSetPos.UP) {
            targetArmPos[1] = (int) (600);
            if (robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() < 650) {
                upPosTracking = PosWait.GO;
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

        // Camera color toggle
        if (gamepad1.back && grabSampleTimer.milliseconds() > 200) {
            if (cameraColorTracking == cameraColor.YELLOW) {
                cameraColorTracking = cameraColor.RED;
            } else if (cameraColorTracking == cameraColor.RED) {
                cameraColorTracking = cameraColor.BLUE;
            } else if (cameraColorTracking == cameraColor.BLUE) {
                cameraColorTracking = cameraColor.YELLOW;
            }
            grabSampleTimer.reset();
            cameraColorUpdate();
        }

        // Update robot systems
        rotationalSensitivity = 2;  // Adjust this for desired sensitivity
        rotationalExponentialOutput = Math.signum(-gamepad1.right_stick_x) * Math.pow(Math.abs(gamepad1.right_stick_x), rotationalSensitivity);
        rotationalForwardOffset = (0);
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x * -0.46, true);
        follower.update();
        updateArmLimits();
        robotCoreCustom.updateAll();

        // Telemetry for debugging
        telemetryA.addData("cameraColor", cameraColorTracking);
        telemetryA.addData("gripperRotationPosTarget", gripperRotationPosTarget);
        telemetryA.addData("gripperRotationStateTracking", gripperRotationStateTracking);
        telemetryA.addData("cameraFrameCount", cameraFrameCount);
        telemetryA.addData("cameraStateTracking", cameraStateTracking);
        telemetryA.addData("rectangleInView", cameraCoreCustom.isRectangleInView());

        try {
            telemetryA.addData("computedAngle", cameraCoreCustom.getComputedAngle());
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

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
        if ((gamepad.left_bumper && gripperRotationStateTracking == gripperRotationState.MANUAL) && gripperTimer.milliseconds() > 50) {
            gripperTimer.reset();
            gripperRotationPosTarget = gripperRotationPosTarget - 0.007;
        }
        if ((gamepad.right_bumper && gripperRotationStateTracking == gripperRotationState.MANUAL) && gripperTimer.milliseconds() > 50) {
            gripperTimer.reset();
            gripperRotationPosTarget = gripperRotationPosTarget + 0.007;
        }
        if (gripperRotationPosTarget > 1) gripperRotationPosTarget = 1;
        if (gripperRotationPosTarget < 0) gripperRotationPosTarget = 0;

        // Toggle gripper open/close
        if (gamepad.a && gripperTimer.milliseconds() > 160) {
            gripperTimer.reset();
            gripperTracking = (gripperTracking == gripperPos.OPEN) ? gripperPos.CLOSE : gripperPos.OPEN;
        }

        // Toggle gripper pitch
        if (gamepad.b && gripperTimer.milliseconds() > 200) {
            gripperTimer.reset();
            if (gripperPitchTracking == gripperPitchPos.FORWARD) gripperPitchTracking = gripperPitchPos.BACKWARD;
            else if (gripperPitchTracking == gripperPitchPos.BACKWARD) gripperPitchTracking = gripperPitchPos.FORWARD;
            else if (gripperPitchTracking == gripperPitchPos.MIDDLE) gripperPitchTracking = gripperPitchPos.BACKWARD;
        }
        if (gamepad.y && gripperTimer.milliseconds() > 200) {
            gripperTimer.reset();
            gripperPitchTracking = gripperPitchPos.MIDDLE;
        }

        // Update hardware states based on tracking
        if (gripperPitchTracking == gripperPitchPos.FORWARD) robotCoreCustom.setGripperPitch(1); //1
        else if (gripperPitchTracking == gripperPitchPos.MIDDLE) robotCoreCustom.setGripperPitch(0.7); //0.7
        else if (gripperPitchTracking == gripperPitchPos.BACKWARD) robotCoreCustom.setGripperPitch(0.3); // 0.3

        robotCoreCustom.setGripper(
                (gripperTracking == gripperPos.OPEN) ? 0.8 : 0.45 // Adjust positions for open/close
        );
        robotCoreCustom.setGripperRotation(gripperRotationPosTarget);
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

    // Updates camera rotation based on target detection
    public void cameraRotationUpdate() throws Exception {
        if (cameraStateTracking == cameraState.SEARCHING) {
            gripperRotationStateTracking = gripperRotationState.AUTO;
            cameraUpdateTimer.reset();
            if (cameraCoreCustom.isRectangleInView()) {
                cameraStateTracking = cameraState.FOUND;
            }
            if (cameraWasFound && cameraResetTimer.milliseconds() > 500) {
                rotationDataProcessor.clearDataPoints();
                cameraResetTimer.reset();
                cameraFrameCount = 0;
                cameraWasFound = false;  // Add this
            }
        }
        if (cameraStateTracking == cameraState.FOUND) {
            if (!cameraCoreCustom.isRectangleInView()) {  // This should be checking for lost target
                cameraStateTracking = cameraState.SEARCHING;
            } else {
                if (cameraCoreCustom.getComputedAngle() == 0) return;
                rotationDataProcessor.addDataPoint(cameraCoreCustom.getComputedAngle());
                cameraWasFound = true;
                cameraResetTimer.reset();
                gripperRotationPosTarget = Math.abs(cameraCoreCustom.getComputedAngle());
            }
        }
        if (cameraStateTracking == cameraState.AVERAGING && cameraUpdateTimer.milliseconds() > 3) {
            cameraUpdateTimer.reset();
            rotationDataProcessor.removeOutliers();
            cameraStateTracking = cameraState.SEARCHING;
            gripperRotationStateTracking = gripperRotationState.MANUAL;
        }
    }

    // Updates camera state based on button input
    public void cameraButtonUpdate() {
        if (cameraButtonTimer.milliseconds() > 500 && gamepad2.x) {
            cameraButtonTimer.reset();
            cameraStateTracking = cameraState.SEARCHING;
        }
    }

    // Updates camera color pipeline
    public void cameraColorUpdate() {
        if (cameraColorTracking == cameraColor.YELLOW) {
            cameraCoreCustom.setPipeline(0);
        }
        if (cameraColorTracking == cameraColor.RED) {
            cameraCoreCustom.setPipeline(1);
        }
        if (cameraColorTracking == cameraColor.BLUE) {
            cameraCoreCustom.setPipeline(2);
        }
    }

    // Handles sample pickup sequence
    public void pickupSample() {
        if (gripperGrabSamplePos == 0) {
            gripperPitchTracking = gripperPitchPos.FORWARD;
            gripperTracking = gripperPos.OPEN;
            gripperGrabSamplePos = 1;
            grabSampleTimer.reset();
        }
        if (gripperGrabSamplePos == 1 && grabSampleTimer.milliseconds() > 250) {
            gripperTracking = gripperPos.CLOSE;
            gripperGrabSamplePos = 2;
        }
        if (gripperGrabSamplePos == 2 && grabSampleTimer.milliseconds() > 100) {
            gripperPitchTracking = gripperPitchPos.MIDDLE;
            gripperGrabSamplePos = 0;
        }
    }
}