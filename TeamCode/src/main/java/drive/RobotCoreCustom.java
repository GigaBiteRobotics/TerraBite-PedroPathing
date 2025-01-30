package drive;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotCoreCustom {
    // Variables and instances for inverse kinematics and motor control
    InverseKinematics2D inverseKinematics2D = new InverseKinematics2D(0, 0, 0, 10000, 0);
    public MotorController motorControllerExt0 = new MotorController();
    public MotorController motorControllerExt1 = new MotorController();
    public MotorController motorControllerRot = new MotorController();
    // Sensors and limit switches
    public TouchSensor extLimitSwitch, rotLimitSwitch;

    // Homing states for extension and rotation
    public HomingState extHomingState = HomingState.IDLE;
    public HomingState rotHomingState = HomingState.IDLE;
    ElapsedTime sweeperTimer = new ElapsedTime();
    // PID coefficients for motors
    public PIDFCoefficients pidfCoefficientsExt = new PIDFCoefficients(20, 1, 0.2, 0.3);
    public PIDFCoefficients pidfCoefficientsRot = new PIDFCoefficients(0, 0, 0, 0);
    public CustomPIDFController extPIDFController = new CustomPIDFController(20, 8, 0, 0);
    public int extTicks = 0;
    public double targetExtPower;

    // Homing power for motors
    private final double rotHomingPower = -0.8;
    private final double extHomingPower = -1;
    // Servo components for the gripper
    public Servo gripperRoller0, gripperRoller1, gripperPitch, gripper, sweeperServo;
    // Enum for homing states
    public enum HomingState {
        IDLE,
        HOMING,
        SUCCESS,
    }

    // Enum for servo roller direction
    public enum Direction {
        FORWARD,
        REVERSE
    }

    /**
     * Initializes all hardware components of the robot.
     * @param hardwareMap The hardware map from the OpMode.
     */
    public void robotCoreInit(HardwareMap hardwareMap) {
        // Initialize motors
        motorControllerExt0.initializeMotor("armExt0", hardwareMap, false, false, pidfCoefficientsExt, false);
        motorControllerExt1.initializeMotor("armExt1", hardwareMap, false, false, pidfCoefficientsExt, false);
        motorControllerRot.initializeMotor("armRot", hardwareMap, false, false, pidfCoefficientsRot, true);
        motorControllerExt1.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize limit switches
        extLimitSwitch = hardwareMap.get(TouchSensor.class, "extLimit");
        rotLimitSwitch = hardwareMap.get(TouchSensor.class, "rotLimit");

        // Initialize servos
        gripper = hardwareMap.get(Servo.class, "gripper");
        gripperPitch = hardwareMap.get(Servo.class, "gripperPitch");
        gripperRoller0 = hardwareMap.get(Servo.class, "gripperRoller0");
        gripperRoller1 = hardwareMap.get(Servo.class, "gripperRoller1");
        sweeperServo = hardwareMap.get(Servo.class, "sweeperServo");
    }

    /**
     * Calculates inverse kinematics positions for the arm.
     * @param pos A double array with x and y positions.
     * @return A double array with the calculated positions, or null if invalid.
     */
    public double[] calcIKPos(double[] pos) {
        return inverseKinematics2D.solve(pos[0], pos[1]);
    }

    /**
     * Computes the maximum arm extension length based on rotation angle.
     * @param angle The rotation angle in degrees.
     * @return The maximum length in inches.
     */
    public double getMaxLength(double angle) {
        return inverseKinematics2D.computeMaxLength(angle);
    }

    /**
     * Sets the arm extension to a target position.
     * @param length The desired length in inches.
     */
    public void setArmExtPos(double length) {
        extTicks = (int) length; //((length / 11.3) * 537.7); older equation to find conversion// Convert length to motor ticks

        /*
        motorControllerExt0.setMotorPos(ticks, 1);
        motorControllerExt1.setMotorPos(0, 0);
         */
    }
    public void setArmPos(int[] pos) {
        setArmExtPos(pos[1]);
        setArmRotPos(pos[0], 1);
    }

    /**
     * Sets the arm rotation to a target angle.
     * @param ticks The desired angle in encoder ticks.
     */
    public void setArmRotPos(double ticks, double speed) {
        //int ticks = (int) (angle * 1.49361111 * 3); // Convert angle to motor ticks
        motorControllerRot.setMotorPos((int) ticks, speed);
    }
    public void setArmRotVel(double vel) {
        int currentPos = motorControllerRot.motor.getCurrentPosition();
        if (currentPos > 2500) {
            if (vel > 0) {
                vel = 0;
            }
        }
        if (currentPos < 22) {
            if (vel < 0) {
                vel = 0;
            }
        }
        motorControllerRot.motor.setPower(vel);
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
     * Initiates the homing process for the arm extension.
     */
    public void homeExt() {
        if (extHomingState == HomingState.IDLE || extHomingState == HomingState.SUCCESS) {
            extHomingState = HomingState.HOMING;
            motorControllerExt0.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorControllerExt1.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorControllerExt0.motor.setPower(extHomingPower);
            motorControllerExt1.motor.setPower(extHomingPower);
        }
    }

    /**
     * Initiates the homing process for the arm rotation.
     */
    public void homeRot() {
        if (rotHomingState == HomingState.IDLE || rotHomingState == HomingState.SUCCESS) {
            rotHomingState = HomingState.HOMING;
            motorControllerRot.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorControllerRot.motor.setPower(rotHomingPower);
        }
    }

    /**
     * Updates the homing state based on limit switch readings.
     */
    public void updateHoming() {
        if (extHomingState == HomingState.HOMING && extLimitSwitch.isPressed()) {
            motorControllerExt0.motor.setPower(0);
            motorControllerExt1.motor.setPower(0);
            resetEncoders(motorControllerExt0, motorControllerExt1);
            motorControllerExt0.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorControllerExt1.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extHomingState = HomingState.SUCCESS;
        }
        if (rotHomingState == HomingState.HOMING && rotLimitSwitch.isPressed()) {
            motorControllerRot.motor.setPower(0);
            resetEncoders(motorControllerRot);
            rotHomingState = HomingState.SUCCESS;
            //motorControllerRot.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    /**
     * Resets motor encoders and sets them to run using encoders.
     */
    private void resetEncoders(MotorController... motors) {
        for (MotorController motor : motors) {
            motor.motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    /**
     * Sets the gripper rollers' direction and speed.
     * When direction is FORWARD, servos are set towards position 1 with speed adjustment.
     * When direction is REVERSE, servos are set towards position 0 with speed adjustment.
     * @param speed The speed value (0 to 1) to control the intensity.
     * @param direction The direction (FORWARD or REVERSE).
     */
    public void setGripperRollers(double speed, Direction direction) {
        if (direction == Direction.REVERSE) {
            gripperRoller1.setPosition(1 * speed);
            gripperRoller0.setPosition(((speed)*-1)+1);
        } else {
            gripperRoller0.setPosition(1 * speed);
            gripperRoller1.setPosition(((speed) * -1) + 1);
        }
    }

    /**
     * Sets the gripper pitch servo position.
     * @param position The desired position (0 to 1).
     */
    public void setGripperPitch(double position) {
        gripperPitch.setPosition(position);
    }

    /**
     * Sets the gripper servo position.
     * @param position The desired position (0 to 1).
     */
    public void setGripper(double position) {
        gripper.setPosition(position);
    }
    public void sweep() {
        sweeperServo.setPosition(0.9);
        sweeperTimer.reset();
    }
    public void updateSweep() {
        if (sweeperTimer.milliseconds() > 800) {
            sweeperServo.setPosition(0.07);
        }
    }
    public void updatePIDF() {
        if (extHomingState == HomingState.IDLE || extHomingState == HomingState.SUCCESS) {
            targetExtPower = extPIDFController.calculate(extTicks, motorControllerExt0.motor.getCurrentPosition(), 0, 20);
            motorControllerExt0.motor.setPower(targetExtPower);
            motorControllerExt1.motor.setPower(targetExtPower);
        }
    }
    public void updateAll() {
        updateSweep();
        updateHoming();
        updatePIDF();
    }
}
