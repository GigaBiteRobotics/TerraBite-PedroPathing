package gigabite.robots.minibot.actions;

import gigabite.actions.AndAction;

// Turn the robot in place either to the right or left
// by adding two actions, one for each drive wheel.
public class TurnAction extends AndAction {
public enum Direction {
        Left,
        Right,
    };
    public TurnAction(String name, double seconds, double power, Direction direction) {
        super(name);
        if(direction == Direction.Left) {
            actions_.add( new DriveAction("TurnLeft", DriveAction.DriveWheel.Left, power, seconds));
            actions_.add( new DriveAction("TurnRight", DriveAction.DriveWheel.Right, -power, seconds));
        } else {
            actions_.add( new DriveAction("TurnLeft", DriveAction.DriveWheel.Left, -power, seconds));
            actions_.add( new DriveAction("TurnRight", DriveAction.DriveWheel.Right, power, seconds));
        }
    }
}
