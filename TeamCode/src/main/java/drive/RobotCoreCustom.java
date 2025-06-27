package drive;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class RobotCoreCustom {
    public MotorController motorControllerExt0 = new MotorController();
    public MotorController motorControllerExt1 = new MotorController();
    public MotorController motorControllerRot0 = new MotorController();
    public MotorController motorControllerRot1 = new MotorController();
    // Homing states for extension and rotation
    public HomingState extHomingState = HomingState.IDLE;
    public HomingState rotHomingState = HomingState.IDLE;
    public currentState extCurrentState = currentState.UNKNOWN;
    public currentState rotCurrentState = currentState.UNKNOWN;
    private VoltageSensor controlHubVoltageSensor;
    public TouchSensor upLimitSW, downLimitSW, extLimitSW;
	// Servo components for the gripper
    public Servo gripper, servoDiffLeft, servoDiffRight, servoWristRight, servoWristLeft;
    CustomPIDFController pidfControllerExt = new CustomPIDFController(110, 10, 2.5, 7);
    public boolean enablePIDFExt = true; // Enable PIDF control for extension
    // Enum for homing states
    public enum HomingState {
        IDLE,
        DOWN,
        UP,
        SUCCESS
    }
    public enum currentState {
        UNKNOWN,
        DOWN,
        UP,
        CONTROLLED
    }

    /**
     * Initializes all hardware components of the robot.
     * @param hardwareMap The hardware map from the OpMode.
     */
    public void robotCoreInit(HardwareMap hardwareMap) {
        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        upLimitSW = hardwareMap.get(TouchSensor.class, "backLimitSwitch");
        downLimitSW = hardwareMap.get(TouchSensor.class, "downLimitSwitch");
        extLimitSW = hardwareMap.get(TouchSensor.class, "extLimitSwitch");

        // Initialize motors
        motorControllerRot0.initializeMotor("expM2", hardwareMap, false, true, null, false);
        motorControllerRot1.initializeMotor("expM3", hardwareMap, false, false, null, false);
        motorControllerExt0.initializeMotor("expM0", hardwareMap, false, false, null, false);
        motorControllerExt1.initializeMotor("expM1", hardwareMap, false, true, null, false);
        motorControllerRot0.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorControllerRot1.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize servos
        servoDiffLeft = hardwareMap.get(Servo.class, "servo4");
        servoDiffRight = hardwareMap.get(Servo.class, "servo0");
        servoWristLeft = hardwareMap.get(Servo.class, "servo3");
        servoWristRight = hardwareMap.get(Servo.class, "servo1");
        gripper = hardwareMap.get(Servo.class, "servo2"); // OPEN: 0.35, CLOSE: 0.57
        rotHomingState = HomingState.IDLE;
        extHomingState = HomingState.IDLE;
    }
    public void homeMotorExt() {
        if (extHomingState == HomingState.IDLE || extHomingState == HomingState.SUCCESS) {
            extHomingState = HomingState.DOWN;
        }
    }
    public void homeMotorRot(HomingState direction) {
        if (direction == HomingState.DOWN && (rotHomingState == HomingState.IDLE)) {
            rotHomingState = HomingState.DOWN;
        } else if (direction == HomingState.UP && (rotHomingState == HomingState.IDLE)) {
            rotHomingState = HomingState.UP;
        }
    }
    public void homingUpdate() {
        double voltagePowerMap = 1;   //Math.min(1, 4.0 / controlHubVoltageSensor.getVoltage()); // More sensitive at lower voltage, max 1
        if (extHomingState == HomingState.DOWN) {
            if (extLimitSW.isPressed()) {
                // If the current is above a threshold, we assume the limit switch is pressed
                extHomingState = HomingState.SUCCESS;
                motorControllerExt0.motor.setPower(0);
                motorControllerExt1.motor.setPower(0);
                motorControllerExt0.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorControllerExt1.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorControllerExt0.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorControllerExt1.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                extCurrentState = currentState.DOWN;

            } else {
                // Continue homing logic here, e.g., moving motors until limit switch is pressed
                motorControllerExt0.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorControllerExt1.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorControllerExt0.motor.setPower(-0.5);
                motorControllerExt1.motor.setPower(-0.5);
            }
        }
        if (extLimitSW.isPressed()) {
            // If the current is above a threshold, we assume the limit switch is pressed
            extHomingState = HomingState.SUCCESS;
            motorControllerExt0.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorControllerExt1.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorControllerExt0.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorControllerExt1.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extCurrentState = currentState.DOWN;
        } else {extCurrentState = currentState.UNKNOWN;}

        if (rotHomingState == HomingState.DOWN) {
            if (downLimitSW.isPressed()) {
                rotHomingState = HomingState.IDLE;
                motorControllerRot0.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorControllerRot1.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorControllerRot0.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorControllerRot1.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorControllerRot0.motor.setPower(0);
                motorControllerRot1.motor.setPower(0);
                rotCurrentState = currentState.DOWN;
            } else {
                motorControllerRot0.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorControllerRot1.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorControllerRot0.motor.setPower(-1);
                motorControllerRot1.motor.setPower(-1);
            }
        }
        if (rotHomingState == HomingState.UP) {
            if (upLimitSW.isPressed()) {
                rotHomingState = HomingState.IDLE;
                motorControllerRot0.motor.setPower(0);
                motorControllerRot1.motor.setPower(0);
                rotCurrentState = currentState.UP;
            } else {
                motorControllerRot0.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorControllerRot1.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorControllerRot0.motor.setPower(1);
                motorControllerRot1.motor.setPower(1);
            }
        }
    }

    public void setExtPos(double position) {
        if (rotCurrentState == currentState.DOWN && position > 60) {
            position = 60;
        }
        if (!enablePIDFExt) {
            return;
        }
        double power = 0;
        if (extHomingState == HomingState.SUCCESS) {
            if (rotCurrentState == currentState.DOWN) {
                // If the extension is homed and the rotation is down, we can set the position
                power = Math.max(-1, Math.min(1, pidfControllerExt.calculate(position, -motorControllerExt0.motor.getCurrentPosition(), 0, 10))); // Encoder Reversed and Clamped to prevent slamming
            } else if (rotCurrentState == currentState.UP && -motorControllerExt0.motor.getCurrentPosition() < 300) {
                // If the extension is homed and the rotation is up, we can set the position
                power = Math.max(-0.2, Math.min(1, pidfControllerExt.calculate(position, -motorControllerExt0.motor.getCurrentPosition(), 0, 10))); // Encoder Reversed and Clamped to prevent slamming
            } else {
                power = Math.max(-1, Math.min(1, pidfControllerExt.calculate(position, -motorControllerExt0.motor.getCurrentPosition(), 0, 10))); // Encoder Reversed and Clamped to prevent slamming
            }
            motorControllerExt0.motor.setPower(power);
            motorControllerExt1.motor.setPower(power);
        }
    }

    /**
     * Checks if a path is finished based on the follower's current position.
     * @param follower The path follower instance
     * @return True if the path is completed, otherwise false.
     */
    public boolean isPathFinished(Follower follower, Pose targetPose) {
        double poseX = follower.getPose().getX();
        double poseY = follower.getPose().getY();
        return (Math.abs(poseX - targetPose.getX()) < 3) && (Math.abs(poseY - targetPose.getY()) < 3);
        //if (poseX > (targetPose.getX() - 1) && poseY > (targetPose.getY() - 1)) return true;
        //return false;
    }

    /**
     * Sets the gripper servo position.
     * @param position The desired position (0 to 1).
     */
    public void setGripper(double position) {
        gripper.setPosition(position);
    }

    public void setDiffPos(double[] pos) {
        double pitchComponent = pos[0]; // -1 to 1
        double rotation = pos[1];    // -1 to 1

        double rotationComponent = pitchComponent * (18.0 / 52.0); // 18/52

        // Increase pitch range: scale pitch by 1.0 instead of 0.5
        double servoLeft = 0.5 + (rotationComponent * 0.5) + (rotation * 0.5);
        double servoRight = 0.5 - (rotationComponent * 0.5) + (rotation * 0.5);

        // Clamp to [0, 1]
        servoLeft = Math.max(0, Math.min(1, servoLeft));
        servoRight = Math.max(0, Math.min(1, servoRight));

        // Set the servo positions
        servoDiffLeft.setPosition(servoLeft-0.02);
        servoDiffRight.setPosition(servoRight-0.02);
    }

    /**
     * Sets the wrist servo position.
     * @param position The desired position (0 to 1).
     */
    public void setWrist(double position) {
        servoWristLeft.setPosition(position);
        servoWristRight.setPosition(1-position);
    }
    /**
     * Updates all components of the robot.
     */
    public void updateAll() {
        homingUpdate();
    }
}