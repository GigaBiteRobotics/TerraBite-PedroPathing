package gigabite.robots.minibot.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import gigabite.robot.Robot;
import gigabite.robots.minibot.MiniBot;
import gigabite.robots.minibot.TankDriver;
import gigabite.robots.minibot.PovDriver;
import gigabite.robot.RobotContext;
import gigabite.robots.minibot.actions.SnakeAction;
import gigabite.robots.minibot.actions.TurnAction;


@TeleOp(name = "TankBot.Drive", group = "Linear OpMode")
@Disabled
public class DriveOp extends OpMode {
    RobotContext context = null;

    @Override
    public void init() {
        context = new RobotContext();
        context.opMode = this;
        context.elapsedTime = new ElapsedTime();
        context.robot = new MiniBot(context);

        // choose a pov or tank driver for human control
        // robot.AddDriver(new TankDriver(context));
        context.robot.AddDriver(new PovDriver(context));
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
        context.robot.update();

        // select a different drive mode
        if(context.opMode.gamepad1.dpad_up) {
            context.robot.ClearDrivers();
            context.robot.AddDriver(new PovDriver(context));
        }
        if(context.opMode.gamepad1.dpad_down) {
            context.robot.ClearDrivers();
            context.robot.AddDriver(new TankDriver(context));
        }

        // test out some automated actions.
        if(context.opMode.gamepad1.circle && context.robot.CurrentAction() == null) {
            context.robot.AddAction( new TurnAction("LeftTurn", 5.0, 1.0, TurnAction.Direction.Left));
        }
        if(context.opMode.gamepad1.square && context.robot.CurrentAction() == null) {
            context.robot.AddAction( new SnakeAction("Snake", 5.0, 4.0, 1.0));
        }
    }
}
