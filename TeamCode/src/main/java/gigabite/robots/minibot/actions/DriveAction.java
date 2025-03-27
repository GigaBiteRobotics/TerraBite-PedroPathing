package gigabite.robots.minibot.actions;

import gigabite.actions.Action;
import gigabite.actions.WaitAction;
import gigabite.actions.ActionContext;
import gigabite.robots.minibot.MiniBot;

public class DriveAction extends WaitAction {
    public enum DriveWheel {
        Left,
        Right
    };

    private DriveWheel wheel_;
    private double power_;
    // activate one of the wheels for a certain amount of time.
    public DriveAction(String name, DriveWheel wheel, double power, double seconds) {
        super(name, seconds);
        wheel_ = wheel;
        power_ = power;
    }

    public Action.Status start(ActionContext context) {
        super.start(context);
        MiniBot robot = (MiniBot)context.robot;
        if(wheel_ == DriveWheel.Left) {
            robot.setLeftDrive(power_);
        } else {
            robot.setRightDrive(power_);
        }
        return Status.Success;
    }
    @Override
    public Status stop(ActionContext context) {
        super.stop(context);
        MiniBot robot = (MiniBot)context.robot;
        if(wheel_ == DriveWheel.Left) {
            robot.setLeftDrive(0);
        } else {
            robot.setRightDrive(0);
        }
        return Status.Success;
    }
}
