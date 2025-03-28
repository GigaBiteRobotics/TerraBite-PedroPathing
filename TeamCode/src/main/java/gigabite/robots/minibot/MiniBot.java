package gigabite.robots.minibot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import gigabite.robot.Robot;
import gigabite.robot.RobotContext;

// This is the RevRobotics MiniBot Kit
// See: https://www.revrobotics.com/rev-45-1171/
public class MiniBot extends Robot {
    final private DcMotor leftDrive;
    final private DcMotor rightDrive;

    public MiniBot(RobotContext context) {
        super(context);
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = context_.opMode.hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = context_.opMode.hardwareMap.get(DcMotor.class, "right_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void update() {
        super.update();
        context_.opMode.telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftDrive.getPower(), rightDrive.getPower());
    }

    public void setLeftDrive(double power) {
        power = Range.clip(power, -1.0, 1.0) ;
        leftDrive.setPower(power);
    }
    public void setRightDrive(double power) {
        power = Range.clip(power, -1.0, 1.0) ;
        rightDrive.setPower(power);
    }
}
