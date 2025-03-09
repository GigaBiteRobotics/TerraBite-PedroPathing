package gigabite.robots.tankbot;

import com.qualcomm.robotcore.hardware.DcMotor;

import gigabite.robot.Driver;
import gigabite.robot.RobotContext;

public class MoveDriver extends Driver {

    final private DcMotor leftDrive;
    final private DcMotor rightDrive;
    public MoveDriver(RobotContext context) {
        super(context);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = context.opMode.hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = context.opMode.hardwareMap.get(DcMotor.class, "right_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

    }
    public void update() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        leftPower = -context.opMode.gamepad1.left_stick_y;
        rightPower = -context.opMode.gamepad1.right_stick_y;

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // display wheel power
        context.opMode.telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }
}
