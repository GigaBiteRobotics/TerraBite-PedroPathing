package gigabite.robots.minibot.actions;

import gigabite.actions.Action;
import gigabite.actions.AndAction;
import gigabite.actions.ActionContext;

import gigabite.robots.minibot.MiniBot;
// Turn the robot in place either to the right or left
// by adding two actions, one for each drive wheel.
public class TurnAction extends AndAction {
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

        if(direction_ == Direction.Left) {
            actions_.add( new DriveAction("TurnLeft", DriveAction.DriveWheel.Left, power, seconds));
            actions_.add( new DriveAction("TurnRight", DriveAction.DriveWheel.Right, -power, seconds));
        } else {
            actions_.add( new DriveAction("TurnLeft", DriveAction.DriveWheel.Left, -power, seconds));
            actions_.add( new DriveAction("TurnRight", DriveAction.DriveWheel.Right, power, seconds));
        }
    }
}
