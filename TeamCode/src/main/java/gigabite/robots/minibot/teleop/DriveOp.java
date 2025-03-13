package gigabite.robots.minibot.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import gigabite.robot.Robot;
import gigabite.robots.minibot.MiniBot;
import gigabite.robots.minibot.TankDriver;
import gigabite.robots.minibot.PovDriver;
import gigabite.robot.RobotContext;


@TeleOp(name = "TankBot.Drive", group = "Linear OpMode")
// @Disabled
public class DriveOp extends OpMode {
    Robot robot = null;
    RobotContext context = null;

    @Override
    public void init() {
        context = new RobotContext(this);
        robot = new MiniBot(context);
        // choose a pov or tank driver for human control
        // robot.AddDriver(new TankDriver(context));
        robot.AddDriver(new PovDriver(context));
    }

    @Override
    public void init_loop() {
        // Optional: Logic for repeated initialization before start
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        robot.update();

        // select a different drive mode
        if(context.opMode.gamepad1.dpad_up) {
            robot.ClearDrivers();
            robot.AddDriver(new PovDriver(context));
        }
        if(context.opMode.gamepad1.dpad_down) {
            robot.ClearDrivers();
            robot.AddDriver(new TankDriver(context));
        }
    }
}
