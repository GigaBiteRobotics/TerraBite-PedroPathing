package gigabite.robots.minibot;

import com.qualcomm.robotcore.hardware.DcMotor;

import gigabite.robot.Driver;
import gigabite.robot.RobotContext;

public class TankDriver extends Driver {

    public TankDriver(RobotContext context) {
        super(context);
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
        MiniBot robot = (MiniBot) context.robot;
        if(robot != null) {
            robot.setRightDrive(rightPower);
            robot.setLeftDrive(leftPower);
        }
    }
}
