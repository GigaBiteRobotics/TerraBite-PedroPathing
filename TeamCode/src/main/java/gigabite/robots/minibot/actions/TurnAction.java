package gigabite.robots.minibot.actions;

import gigabite.actions.Action;
import gigabite.actions.ActionContext;

import gigabite.robots.minibot.MiniBot;
public class TurnAction extends Action {
public enum Direction {
        Left,
        Right,
    };
    protected double duration_;
    protected double power_;
    protected Direction direction_;
    public TurnAction(String name, double seconds, double power, Direction direction) {
        super(name);
        duration_ = seconds;
        power_ = power;
        direction_ = direction;
    }

    @Override
    public Action.Status start(ActionContext context) {
        super.start(context);
        MiniBot robot = (MiniBot)context.robot;
        if(direction_ == Direction.Left) {
            robot.setRightDrive(power_);
            robot.setLeftDrive(-power_);
        } else {
            robot.setRightDrive(-power_);
            robot.setLeftDrive(power_);
        }
        return Status.Success;
    }
    @Override
    public Status stop(ActionContext context) {
        super.stop(context);
        MiniBot robot = (MiniBot)context.robot;
        robot.setLeftDrive(0.0);
        robot.setRightDrive(0.0);
        return Status.Success;
    }

    @Override
    public Status update(ActionContext context) {
        super.update(context);
        if (Timer().time() < duration_) {
            return Status.Continue;
        }
        return Status.Success;
    }
}
