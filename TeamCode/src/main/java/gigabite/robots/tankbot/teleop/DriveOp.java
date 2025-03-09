package gigabite.robots.tankbot.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import gigabite.robots.tankbot.TankRobot;
import gigabite.robots.tankbot.MoveDriver;
import gigabite.robot.RobotContext;


@TeleOp(name = "TankBot.Drive", group = "Linear OpMode")
@Disabled
public class DriveOp extends OpMode {
    TankRobot robot = null;
    RobotContext context = null;

    @Override
    public void init() {
        context = new RobotContext(this);
        robot = new TankRobot(context);
        robot.AddDriver(new MoveDriver(context));
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
    }
}
