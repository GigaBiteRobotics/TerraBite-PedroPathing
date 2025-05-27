package gigabite.robots.minibot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import gigabite.robot.Driver;
import gigabite.robot.RobotContext;

public class PovDriver extends Driver {
    final private DcMotor leftDrive = null;
    final private DcMotor rightDrive = null;
    public PovDriver(RobotContext context) {
        super(context);
    }

    public void update() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -context.opMode.gamepad1.left_stick_y;
        double turn  =  context.opMode.gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
        MiniBot robot = (MiniBot) context.robot;
        if(robot != null) {
            robot.setRightDrive(rightPower);
            robot.setLeftDrive(leftPower);
        }
    }
}
