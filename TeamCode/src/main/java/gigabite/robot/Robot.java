package gigabite.robot;

import com.qualcomm.robotcore.util.ElapsedTime;
import gigabite.actions.Action;
import gigabite.actions.ActionContext;

// base class for all robots
// Basic Idea : Robots perform Actions.
public class Robot {
    // member variables
    protected Action action_ = null;
    protected ActionContext actionContext_ = null;

    public Robot() {
        actionContext_ = new ActionContext();
        actionContext_.elapsedTime = new ElapsedTime();
    }

    // inteface
    //update the robot
    public void update() {
        if(action_ != null) {
            if(action_.update(actionContext_) != Action.Status.Continue) {
                stopAction();
            }
        }
    }

    // start running this action
    public void runAction(Action a) {
        action_ = a;
        if(action_.start(actionContext_) == Action.Status.Failed) {
            // TODO: report failure
            stopAction();
        }
    }

    // stop running the current action
    public void stopAction() {
        if (action_ != null) {
            if(action_.stop(actionContext_) == Action.Status.Failed) {
                // TODO: report failure
            }
        }
    }
    // query the current action
    public Action currentAction() {
        return action_;
    }
}
