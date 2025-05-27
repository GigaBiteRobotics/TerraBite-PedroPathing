package gigabite.robots.minibot.actions;

import gigabite.actions.AndAction;

// Moves the robot forwards ( or backwards ) for a certain amount of time
public class MoveAction extends AndAction {
    public MoveAction(String name, double seconds, double power) {
        super(name);
        actions_.add( new DriveAction("LeftWheel", DriveAction.DriveWheel.Left, power, seconds));
        actions_.add( new DriveAction("RightWheel", DriveAction.DriveWheel.Right, power, seconds));
    }
}
