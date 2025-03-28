package gigabite.robots.minibot.actions;

import gigabite.actions.ActionList;
import gigabite.actions.WaitAction;

public class DanceAction extends ActionList {

    public DanceAction(String name) {
        super(name);
        add(new SnakeAction("snaky", 5.0, 5,  1.0));
        add(new MoveAction("backward", 1.0, -0.75));
        add(new TurnAction("Left", 1.0, 1, TurnAction.Direction.Left));
        add(new WaitAction("wait", 0.25));
        add(new TurnAction("Right", 1.0, 1, TurnAction.Direction.Right));
        add(new WaitAction("wait", 0.5));
        add(new MoveAction("forwards", 1.0, 1.0));
        add(new MoveAction("backwards", 1.0, -1.0));
        add(new WaitAction("wait", 0.75));
        add(new SnakeAction("snaky", 5.0, 5, 0.5));
        add(new MoveAction("backwards", 1.0, -0.75));
    }
}
