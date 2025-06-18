package drive;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class RobotCoreCustom {
    public MotorController motorControllerExt0 = new MotorController();
    public MotorController motorControllerExt1 = new MotorController();
    public MotorController motorControllerRot0 = new MotorController();
    public MotorController motorControllerRot1 = new MotorController();
    // Homing states for extension and rotation
    public HomingState extHomingState = HomingState.IDLE;
    public HomingState rotHomingState = HomingState.IDLE;
    private VoltageSensor myControlHubVoltageSensor;
	// Servo components for the gripper
    public Servo gripper, servoDiffLeft, servoDiffRight, servoWristRight, servoWristLeft;
    CustomPIDFController pidfControllerExt = new CustomPIDFController(110, 10, 2.5, 7);
    // Enum for homing states
    public enum HomingState {
        IDLE,
        DOWN,
        UP,
        SUCCESS
    }

    /**
     * Initializes all hardware components of the robot.
     * @param hardwareMap The hardware map from the OpMode.
     */
    public void robotCoreInit(HardwareMap hardwareMap) {
        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        // Initialize motors
        motorControllerRot0.initializeMotor("expM2", hardwareMap, false, true, null, false);
        motorControllerRot1.initializeMotor("expM3", hardwareMap, false, false, null, false);
        motorControllerExt0.initializeMotor("expM0", hardwareMap, false, false, null, false);
        motorControllerExt1.initializeMotor("expM1", hardwareMap, false, false, null, false);
        motorControllerRot0.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorControllerRot1.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize servos
        servoDiffLeft = hardwareMap.get(Servo.class, "servoDiffLeft");
        servoDiffRight = hardwareMap.get(Servo.class, "servoDiffRight");
        servoWristLeft = hardwareMap.get(Servo.class, "servoWristLeft");
        servoWristRight = hardwareMap.get(Servo.class, "servoWristRight");
        gripper = hardwareMap.get(Servo.class, "gripper");
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
        if (extHomingState == HomingState.DOWN) {
            if (motorControllerExt0.motor.getCurrent(CurrentUnit.AMPS) > 6 || motorControllerExt1.motor.getCurrent(CurrentUnit.AMPS) > 6) {
                // If the current is above a threshold, we assume the limit switch is pressed
                extHomingState = HomingState.SUCCESS;
                motorControllerExt0.motor.setPower(0);
                motorControllerExt1.motor.setPower(0);
                motorControllerExt0.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorControllerExt1.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorControllerExt0.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorControllerExt1.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            } else {
                // Continue homing logic here, e.g., moving motors until limit switch is pressed
                motorControllerExt0.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorControllerExt1.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorControllerExt0.motor.setPower(-0.7);
                motorControllerExt1.motor.setPower(-0.7);
            }

        }
        if (rotHomingState == HomingState.DOWN) {
            if (motorControllerRot0.motor.getCurrent(CurrentUnit.AMPS) > 2.7 || motorControllerRot1.motor.getCurrent(CurrentUnit.AMPS) > 2.7) {
                // If the current is above a threshold, we assume the limit switch is pressed
                rotHomingState = HomingState.IDLE;
                motorControllerRot0.motor.setPower(0);
                motorControllerRot1.motor.setPower(0);
            } else {
                // Continue homing logic here, e.g., moving motors until limit switch is pressed
                motorControllerRot0.motor.setPower(-1);
                motorControllerRot1.motor.setPower(-1);
            }
        }
        if (rotHomingState == HomingState.UP) {
            if (motorControllerRot0.motor.getCurrent(CurrentUnit.AMPS) > 2.7 || motorControllerRot1.motor.getCurrent(CurrentUnit.AMPS) > 2.7) {
                // If the current is above a threshold, we assume the limit switch is pressed
                rotHomingState = HomingState.IDLE;
                motorControllerRot0.motor.setPower(0);
                motorControllerRot1.motor.setPower(0);
            } else {
                // Continue homing logic here, e.g., moving motors until limit switch is pressed
                motorControllerRot0.motor.setPower(1);
                motorControllerRot1.motor.setPower(1);
            }
        }
    }

    public void setExtPos(double position) {
        double power = 0;
        if (extHomingState == HomingState.SUCCESS) {
            if (-motorControllerExt0.motor.getCurrentPosition() > 100){
                power = Math.max(-0.8, Math.min(1, pidfControllerExt.calculate(position, -motorControllerExt0.motor.getCurrentPosition(), 0, 10))); //  Encoder Reversed and Clamped to prevent slamming
            } else if (motorControllerExt0.motor.getVelocity(AngleUnit.DEGREES) > 50) {
                power = Math.max(-0.2, Math.min(1, pidfControllerExt.calculate(position, -motorControllerExt0.motor.getCurrentPosition(), 0, 10))); //  Encoder Reversed and Clamped to prevent slamming
            } else {
                power = Math.max(-1, Math.min(1, pidfControllerExt.calculate(position, -motorControllerExt0.motor.getCurrentPosition(), 0, 10))); //  Encoder Reversed and Clamped to prevent slamming
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
        if (pos == null || pos.length < 2) return;

        double tilt = pos[0];   // [-1, 1]
        double rotate = pos[1]; // [-1, 1]

        // Map to servo range [0, 1]
        double leftPos = tilt + rotate;
        double rightPos = tilt - rotate;

        servoDiffLeft.setPosition(leftPos);
        servoDiffRight.setPosition(rightPos);

    }

    /**
     * Sets the wrist servo position.
     * @param position The desired position (0 to 1).
     */
    public void setWrist(double position) {
        servoWristLeft.setPosition(position);
        servoWristRight.setPosition(position);
    }
    /**
     * Updates all components of the robot.
     */
    public void updateAll() {
        homingUpdate();

    }
}