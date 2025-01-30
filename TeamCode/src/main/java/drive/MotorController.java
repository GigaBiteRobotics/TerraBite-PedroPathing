package drive;

// Importing required classes for motor and PIDF control
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * A class for controlling a motor with optional PIDF settings and encoder-based position control.
 */
public class MotorController {

    // The motor object used for controlling a motor with extended features.
    public DcMotorEx motor;

    // Uncomment and modify this if you want to use a fixed set of PIDF coefficients.
    // public PIDFCoefficients pidfCoefficients = new PIDFCoefficients(20, 0.1, 0.7, 0.5);
    boolean useEncoder;

    /**
     * Initializes the motor with optional PIDF configuration, direction, and encoder reset.
     *
     * @param motorName       The name of the motor as defined in the robot configuration.
     * @param hardwareMap     The hardware map object for accessing hardware devices.
     * @param doPid           Boolean flag to determine whether PIDF control is applied.
     * @param reverse         Boolean flag to reverse motor direction if true.
     * @param pidfCoefficients PIDF coefficients to configure the motor if PIDF control is enabled.
     */
    public void initializeMotor(String motorName, HardwareMap hardwareMap, boolean doPid, boolean reverse, PIDFCoefficients pidfCoefficients, boolean useEncoder) {
        // Retrieve the motor from the hardware map using its name.
        this.motor = hardwareMap.get(DcMotorEx.class, motorName);

        // Reverse the motor's direction if specified.
        if (reverse) this.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset the motor encoder to zero for accurate position tracking.
        this.motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // If PIDF control is enabled, apply the provided PIDF coefficients.
        if (doPid) {
            this.motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        }
        if (useEncoder) this.useEncoder = true;
    }

    /**
     * Sets the motor to move to a specified position with a given power.
     *
     * @param pos   The target position (encoder ticks).
     * @param power The power level to apply to the motor (0.0 to 1.0).
     */
    public void setMotorPos(Integer pos, double power) {
        if (this.useEncoder) {
            // Set the target position for the motor.
            this.motor.setTargetPosition(pos);

            // Set the motor to run to the specified position.
            this.motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // Apply the specified power to the motor.
            this.motor.setPower(power);
        } else this.motor.setPower(0);
    }

    /**
     * Stops the motor by setting its power to zero.
     */
    public void stopMotor() {
        // Stop the motor by setting its power to 0.
        this.motor.setPower(0);
    }
}
