package drive.opmode;

import android.annotation.SuppressLint;

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
import org.opencv.osgi.OpenCVNativeLoader;

import drive.CameraCoreCustom;
import drive.RobotCoreCustom;
import drive.RotationDataProcessorCustom;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name="TerraBiteDrive V1", group="!advanced")
public class MainDriveOpmode extends OpMode{
    RobotCoreCustom robotCoreCustom = new RobotCoreCustom();
    RotationDataProcessorCustom rotationDataProcessor = new RotationDataProcessorCustom(1000);
    CameraCoreCustom cameraCoreCustom = new CameraCoreCustom();
    double[] targetArmPos = {0, 0}; // Arm position: {rotation, extension}
    double[] targetArmVel = {0, 0}; // Arm velocity: {rotation, extension}
    ElapsedTime gamePadPollingRate = new ElapsedTime();
    int pollingRateMS = 5; // Polling rate for gamepad input
    boolean canExtend, canRetract, canTurnPositive, canTurnNegative = true; // Arm limits
    Follower follower;
    Pose startPose = new Pose(7.5, 88.6, 0);
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime grabSampleTimer = new ElapsedTime();
    ElapsedTime cameraButtonTimer = new ElapsedTime();
    ElapsedTime cameraUpdateTimer = new ElapsedTime();
    ElapsedTime cameraResetTimer = new ElapsedTime();
    ElapsedTime scoreTimer = new ElapsedTime();
    double cameraResetTimerMS = 1000;
    boolean cameraWasFound = false;
    int cameraFrameCount = 0;
    // Enums to track gripper states
    public enum gripperPos {
        OPEN,
        CLOSE,
        LIGHT
    }
    public enum gripperPitchPos {
        FORWARD,
        BACKWARD,
        MIDDLE,
        FULLBACK,
        UP,
        GRAB
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
        VEL,
        SCORE
    }
    public enum gripperRotationState {
        AUTO,
        MANUAL
    }
    public enum cameraState {
        SEARCHING,
        AVERAGING,
        FOUND,
        IDLE,
    }
    public enum cameraColor {
        RED,
        BLUE,
        YELLOW
    }
    public enum setPositionType {
        LEFT,
        RIGHT
    }
    setPositionType setPositionTypeTracking = setPositionType.LEFT;
    cameraState cameraStateTracking = cameraState.SEARCHING;
    cameraColor cameraColorTracking = cameraColor.YELLOW;
    gripperRotationState gripperRotationStateTracking = gripperRotationState.MANUAL;
    double rotationalExponentialOutput;
    double rotationalSensitivity;
    double rotationalForwardOffset;
    int gripperGrabSamplePos = 0;
    ElapsedTime grabberTimer = new ElapsedTime();

    // Gripper state tracking variables
    gripperPos gripperTracking;
    gripperPitchPos gripperPitchTracking;
    ElapsedTime gripperTimer;
    PosWait downPosTracking;
    double llXPos;
    double llYPos;
    PosWait upPosTracking;
    targetSetPos targetSetPosTracking = targetSetPos.NA;
    // Telemetry
    Telemetry telemetryA;
    double gripperRotationPosTarget = 0.5;
    ElapsedTime datapointAge = new ElapsedTime();


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
        cameraCoreCustom.limelightCoreCustomInit(hardwareMap);
    }

    @Override
    public void init_loop() {
        // Optional: Logic for repeated initialization before start
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        timer.reset();
        gripperPitchTracking = gripperPitchPos.BACKWARD;
        gripperTracking = gripperPos.CLOSE;
        gripperTimer.reset();
        robotCoreCustom.homeRot();
        datapointAge.reset();
    }

    @Override
    public void loop() {
        // Gamepad input handling
        gamePadCont2DPos(gamepad2);
        gripperChecking(gamepad2);
        //cameraButtonUpdate();
        try {
            cameraRotationUpdate();
        } catch (Exception ignored){}

        if (setPositionTypeTracking == setPositionType.LEFT && gamepad2.back && gripperTimer.milliseconds() > 200) {
            setPositionTypeTracking = setPositionType.RIGHT;
            gripperTimer.reset();
        }
        if (setPositionTypeTracking == setPositionType.RIGHT && gamepad2.back && gripperTimer.milliseconds() > 200) {
            setPositionTypeTracking = setPositionType.LEFT;
            gripperTimer.reset();
        }

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
        if (gamepad2.dpad_down && setPositionTypeTracking == setPositionType.LEFT) {
            targetSetPosTracking = targetSetPos.DOWN;
            downPosTracking = PosWait.GO;
            gripperPitchTracking = gripperPitchPos.FORWARD;

        }
        if (downPosTracking == PosWait.WAIT && targetSetPosTracking == targetSetPos.DOWN && setPositionTypeTracking == setPositionType.LEFT) {
            targetArmPos[1] = (int) (100);
            //targetArmPos[0] = (int) (1200);
            if (robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() < 1600) {
                downPosTracking = PosWait.GO;
            }
        }
        if (downPosTracking == PosWait.GO && targetSetPosTracking == targetSetPos.DOWN && setPositionTypeTracking == setPositionType.LEFT) {
            gripperPitchTracking = gripperPitchPos.MIDDLE;
            targetArmPos[0] = (int) (450);
            targetArmPos[1] = (int) (1800);
            targetSetPosTracking = targetSetPos.NA;
        }

        if (gamepad2.dpad_up && setPositionTypeTracking == setPositionType.LEFT) {
            targetSetPosTracking = targetSetPos.UP;
            gripperRotationPosTarget = 0.474;
            upPosTracking = PosWait.GO;
        }
        if (upPosTracking == PosWait.WAIT && targetSetPosTracking == targetSetPos.UP && setPositionTypeTracking == setPositionType.LEFT) {
            targetArmPos[1] = (int) (600);
            if (robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() < 650) {
                upPosTracking = PosWait.GO;
            }
        }
        if (upPosTracking == PosWait.WAIT1 && targetSetPosTracking == targetSetPos.UP && setPositionTypeTracking == setPositionType.LEFT) {
            targetArmPos[0] = (int) (633);
            if (robotCoreCustom.motorControllerRot.motor.getCurrentPosition() > 600) {
                upPosTracking = PosWait.GO;
            }
        }
        if (upPosTracking == PosWait.GO && targetSetPosTracking == targetSetPos.UP && setPositionTypeTracking == setPositionType.LEFT) {
            targetArmPos[0] = (int) (2184); //2450
            targetArmPos[1] = (int) (5695); //4500
            gripperPitchTracking = gripperPitchPos.FULLBACK;
            targetSetPosTracking = targetSetPos.NA;
        }


        if (gamepad2.dpad_up && setPositionTypeTracking == setPositionType.RIGHT) {
            targetSetPosTracking = targetSetPos.UP;
            upPosTracking = PosWait.WAIT;
        }
        if (upPosTracking == MainDriveOpmode.PosWait.WAIT && targetSetPosTracking == MainDriveOpmode.targetSetPos.UP) {
            targetArmPos[0] = (int) (2250);
            targetArmPos[1] = (int) (0);
            gripperPitchTracking = gripperPitchPos.BACKWARD;
            upPosTracking = MainDriveOpmode.PosWait.GO;
            gripperTracking = MainDriveOpmode.gripperPos.LIGHT;
            gripperTimer.reset();
        }
        if (upPosTracking == MainDriveOpmode.PosWait.GO && targetSetPosTracking == MainDriveOpmode.targetSetPos.UP && gripperTimer.milliseconds() > 750) {
            gripperTracking = MainDriveOpmode.gripperPos.CLOSE;
            targetSetPosTracking = MainDriveOpmode.targetSetPos.NA;
        }

        if (gamepad2.dpad_down && setPositionTypeTracking == setPositionType.RIGHT) {
            targetSetPosTracking = targetSetPos.DOWN;
            downPosTracking = PosWait.GO;
        }
        if (downPosTracking == PosWait.GO && targetSetPosTracking == targetSetPos.DOWN && setPositionTypeTracking == setPositionType.RIGHT) {
            targetArmPos[1] = (int) (100);
            targetArmPos[0] = (int) (1300);
            targetSetPosTracking = targetSetPos.NA;
            gripperPitchTracking = gripperPitchPos.UP;
        }
        if (gamepad2.dpad_down && setPositionTypeTracking == setPositionType.RIGHT) {
            targetSetPosTracking = targetSetPos.DOWN;
            downPosTracking = PosWait.GO;
        }
        if (downPosTracking == PosWait.GO && targetSetPosTracking == targetSetPos.DOWN && setPositionTypeTracking == setPositionType.RIGHT) {
            targetArmPos[1] = (int) (300);
            targetArmPos[0] = (int) (200);
            gripperPitchTracking = gripperPitchPos.GRAB;
            targetSetPosTracking = targetSetPos.NA;
        }

        if (gamepad2.dpad_left && setPositionTypeTracking == setPositionType.RIGHT) {
            targetSetPosTracking = targetSetPos.SCORE;
            downPosTracking = PosWait.WAIT;
        }
        if (downPosTracking == MainDriveOpmode.PosWait.WAIT && targetSetPosTracking == MainDriveOpmode.targetSetPos.SCORE) {
            targetArmPos[0] = (int) (2510);
            targetArmPos[1] = (int) (-10);
            gripperPitchTracking = gripperPitchPos.UP;
            scoreTimer.reset();
            downPosTracking = PosWait.WAIT1;
        }
        if (downPosTracking == MainDriveOpmode.PosWait.WAIT1 && targetSetPosTracking == MainDriveOpmode.targetSetPos.SCORE && scoreTimer.milliseconds() > 350) {
            targetArmPos[0] = (int) (1700);
            gripperTracking = gripperPos.LIGHT;
            scoreTimer.reset();
            downPosTracking = MainDriveOpmode.PosWait.GO;
            scoreTimer.reset();
        }
        if (downPosTracking == MainDriveOpmode.PosWait.GO && targetSetPosTracking == MainDriveOpmode.targetSetPos.SCORE && scoreTimer.milliseconds() > 500) {
            gripperTracking = MainDriveOpmode.gripperPos.OPEN;
            targetSetPosTracking = MainDriveOpmode.targetSetPos.NA;
        }

        if (gripperTimer.milliseconds() > 200 && gamepad2.x || gripperGrabSamplePos == 1 || gripperGrabSamplePos == 2 || gripperGrabSamplePos == 3) {
            if (gripperGrabSamplePos == 0) {
                gripperPitchTracking = gripperPitchPos.FORWARD;
                gripperTracking = gripperPos.OPEN;
                if (gripperRotationStateTracking == gripperRotationState.AUTO) targetArmPos[1] = (int) robotCoreCustom.motorControllerExt0.motor.getCurrentPosition() - 640 - (llYPos - (gripperRotationStateTracking == gripperRotationState.MANUAL ? 150 : 250)); //250
                gripperGrabSamplePos = 1;
                grabSampleTimer.reset();
            }

            if (gripperGrabSamplePos == 1 && grabSampleTimer.milliseconds() > 400) {
                targetArmPos[0] = (int) (120);
                gripperGrabSamplePos = 2;
                grabSampleTimer.reset();
            }
            if (gripperGrabSamplePos == 2 && grabSampleTimer.milliseconds() > 300) {
                gripperTracking = gripperPos.CLOSE;
                gripperGrabSamplePos = 3;
                grabSampleTimer.reset();
            }
            if (gripperGrabSamplePos == 3 && grabSampleTimer.milliseconds() > 200) {
                gripperPitchTracking = gripperPitchPos.MIDDLE;
                gripperGrabSamplePos = 0;
                targetArmPos[0] = (int) (400);
                targetArmPos[1] = (int) (4100);
            }
        }
        if (gamepad2.start && gripperRotationStateTracking == gripperRotationState.MANUAL && grabSampleTimer.milliseconds() > 500) {
            gripperRotationStateTracking = gripperRotationState.AUTO;
            grabSampleTimer.reset();
        } else if (gamepad2.start && gripperRotationStateTracking == gripperRotationState.AUTO && grabSampleTimer.milliseconds() > 500) {
            gripperRotationStateTracking = gripperRotationState.MANUAL;
            grabSampleTimer.reset();
        }

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
        rotationalForwardOffset = ((0));
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x * -0.46, true);
        follower.update();
        updateArmLimits();
        robotCoreCustom.updateAll();
	    try {
            llXPos = cameraCoreCustom.getLLBoxPos()[0];
            llYPos = cameraCoreCustom.getLLBoxPos()[1];
	    } catch (Exception ignore) {}

        // Telemetry for debugging
        telemetryA.addData("cameraColor", cameraColorTracking);
        telemetryA.addData("side", setPositionTypeTracking);
        telemetryA.addData("gripperRotationPosTarget", gripperRotationPosTarget);
        telemetryA.addData("gripperRotationStateTracking", gripperRotationStateTracking);
        telemetryA.addData("armPosExt", robotCoreCustom.motorControllerExt0.motor.getCurrentPosition());
        telemetryA.addData("armPosRot", robotCoreCustom.motorControllerRot.motor.getCurrentPosition());
        telemetryA.addData("armPosExtTarget", targetArmPos[1]);
        telemetryA.addData("armPosRotTarget", targetArmPos[0]);
        telemetryA.addData("gripperPos", gripperPitchTracking);
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
            gripperRotationPosTarget = gripperRotationPosTarget + 0.007;
        }
        if ((gamepad.right_bumper && gripperRotationStateTracking == gripperRotationState.MANUAL) && gripperTimer.milliseconds() > 50) {
            gripperTimer.reset();
            gripperRotationPosTarget = gripperRotationPosTarget - 0.007;
        }
        if (gripperRotationPosTarget > 1) gripperRotationPosTarget = 1;
        if (gripperRotationPosTarget < 0) gripperRotationPosTarget = 0;


        // Toggle gripper open/close
        if (gamepad.a && grabberTimer.milliseconds() > 160) {
            grabberTimer.reset();
            //gripperTracking = (gripperTracking == gripperPos.OPEN) ? gripperPos.CLOSE : gripperPos.OPEN;
            //if (gripperTracking == gripperPos.LIGHT) gripperTracking = gripperPos.OPEN;
            if (gripperTracking == gripperPos.OPEN) gripperTracking = gripperPos.CLOSE;
            else if (gripperTracking == gripperPos.CLOSE) gripperTracking = gripperPos.OPEN;
            else if (gripperTracking == gripperPos.LIGHT) gripperTracking = gripperPos.OPEN;
        }

        // Toggle gripper pitch
        if (gamepad.b && gripperTimer.milliseconds() > 200) {
            gripperTimer.reset();
            if (gripperPitchTracking == gripperPitchPos.FORWARD)
                gripperPitchTracking = gripperPitchPos.BACKWARD;
            else if (gripperPitchTracking == gripperPitchPos.BACKWARD)
                gripperPitchTracking = gripperPitchPos.FORWARD;
            else if (gripperPitchTracking == gripperPitchPos.MIDDLE)
                gripperPitchTracking = gripperPitchPos.BACKWARD;
            else if (gripperPitchTracking == gripperPitchPos.FULLBACK)
                gripperPitchTracking = gripperPitchPos.MIDDLE;
            else if (gripperPitchTracking == gripperPitchPos.UP)
                gripperPitchTracking = gripperPitchPos.BACKWARD;
            else if (gripperPitchTracking == gripperPitchPos.GRAB) {
                gripperPitchTracking = gripperPitchPos.BACKWARD;
            }

        }
        if (gamepad.y && gripperTimer.milliseconds() > 200) {
            gripperTimer.reset();
            gripperPitchTracking = gripperPitchPos.MIDDLE;
        }

        /* Update hardware states based on tracking
        robotCoreCustom.setGripperPitch(
                (gripperPitchTracking == gripperPitchPos.FORWARD) ? 1 : 0.3
                // Adjust pitch as needed
        );
         */

        if (gripperPitchTracking == gripperPitchPos.FORWARD)
            robotCoreCustom.setGripperPitch(1); //1
        else if (gripperPitchTracking == gripperPitchPos.MIDDLE)
            robotCoreCustom.setGripperPitch(0.8); //0.7
        else if (gripperPitchTracking == gripperPitchPos.BACKWARD)
            robotCoreCustom.setGripperPitch(0.25); // 0.3
        else if (gripperPitchTracking == gripperPitchPos.FULLBACK)
            robotCoreCustom.setGripperPitch(0.0);
        else if (gripperPitchTracking == gripperPitchPos.UP)
            robotCoreCustom.setGripperPitch(0.6);
        else if (gripperPitchTracking == gripperPitchPos.GRAB)
            robotCoreCustom.setGripperPitch(0.4);


        if (gripperTracking == gripperPos.OPEN) robotCoreCustom.setGripper(0.65);
        if (gripperTracking == gripperPos.CLOSE) robotCoreCustom.setGripper(0.46);
        if (gripperTracking == gripperPos.LIGHT) robotCoreCustom.setGripper(0.5);
        // Clamp the gripperRotationPosTarget to its allowed range.
        if (gripperRotationPosTarget > 0.5 + (((double) 1 / 1800) * 180))
            gripperRotationPosTarget = 0.5 + (((double) 1 / 1800) * 180);
        if (gripperRotationPosTarget < 0.5 - (((double) 1 / 1800) * 180))
            gripperRotationPosTarget = 0.5 - (((double) 1 / 1800) * 180);

// Ensure gripperRotationPosTarget is a number. If not, reset it.
        if (Double.isNaN(gripperRotationPosTarget)) {
            gripperRotationPosTarget = 0.5;
        }

// Now safely set the servo.
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

            if (targetArmPos[1] < -10) {
                canRetract = false;
                targetArmPos[1] = -10;
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
            if (targetSetPosTracking == targetSetPos.SCORE && downPosTracking == PosWait.GO || downPosTracking == PosWait.WAIT1 || downPosTracking == PosWait.WAIT) {
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
    public void cameraRotationUpdate() throws Exception {
        double datapoint = cameraCoreCustom.getComputedAngle();
        if (rotationDataProcessor.dataPoints.isEmpty()) rotationDataProcessor.addDataPoint(0, datapointAge.milliseconds());
        if (datapoint != 0) {
            rotationDataProcessor.addDataPoint((Math.abs(datapoint) + 40), datapointAge.milliseconds());
        }
        rotationDataProcessor.removeOldDataPoints(15);
        if (cameraStateTracking == cameraState.SEARCHING && gripperRotationStateTracking == gripperRotationState.AUTO && rotationDataProcessor.dataPoints.size() > 10) {
            gripperRotationPosTarget = (Math.abs(rotationDataProcessor.getAverage()) * ((double) 1/1800) + 0.5);
        }
        if (rotationDataProcessor.dataPoints.size() < 5) {
            gripperRotationStateTracking = gripperRotationState.MANUAL;
        }
    }

    public void cameraColorUpdate() {
        if (cameraColorTracking == cameraColor.YELLOW) {
            cameraCoreCustom.setPipeline(0);
        }
        if (cameraColorTracking == cameraColor.RED) {
            cameraCoreCustom.setPipeline(2);
        }
        if (cameraColorTracking == cameraColor.BLUE) {
            cameraCoreCustom.setPipeline(1);
        }
    }
}