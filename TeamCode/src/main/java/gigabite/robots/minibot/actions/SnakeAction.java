package gigabite.robots.minibot.actions;

import com.qualcomm.robotcore.util.Range;

import gigabite.actions.Action;
import gigabite.actions.ActionContext;

import gigabite.robots.minibot.MiniBot;

// Makes the robot move in a snake like fashion
public class SnakeAction extends Action {

    protected double duration_;
    protected double power_;
    protected double period_;

    public SnakeAction(String name, double seconds, double period, double power) {
        super(name);
        duration_ = seconds;
        period_ = period;
        power_ = power;
    }

    @Override
    public Action.Status start(ActionContext context) {
        super.start(context);
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
        MiniBot robot = (MiniBot)context.robot;
        double leftDrive = Math.sin(seconds() * period_) * power_;
        double rightDrive = -leftDrive;
        leftDrive = Range.clip(leftDrive, 0.0, 1.0);
        rightDrive = Range.clip(rightDrive, 0.0, 1.0);
        robot.Context().opMode.telemetry.addData("Snake", " {%f} {%f}", leftDrive, rightDrive);
        robot.setLeftDrive(leftDrive);
        robot.setRightDrive(rightDrive);
        if (Timer().time() < duration_) {
            return Status.Continue;
        }
        return Status.Success;
    }
}
