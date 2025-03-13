package gigabite.robot;


import java.util.ArrayList;

import gigabite.actions.Action;
import gigabite.actions.ActionList;
import gigabite.actions.ActionContext;

// base class for all robots
// Basic Idea : Robots perform Actions.
public class Robot {
    // member variables
    protected Action action_; // current action
    protected ActionList actions_; // list of actions to execute.
    protected ActionContext actionContext_;
    protected RobotContext context_;

    // list of human operated drivers
    protected ArrayList<Driver> drivers_;

    public Robot(RobotContext c) {
        context_ = c;
        actionContext_ = new ActionContext();
        actionContext_.elapsedTime = c.elapsedTime;
        actions_ = new ActionList("actions");
        drivers_ = new ArrayList<Driver>();
    }

    public RobotContext Context() {
        return context_;
    }

    // interface
    //update the robot
    public void update() {
        // update human drivers first, then actions, so that behaviors can override human input
        for (Driver d: drivers_) {
            d.update();
        }
        if(action_ != null) {
            if(action_.update(actionContext_) != Action.Status.Continue) {
                stopAction();
            }
        } else {
            action_ = actions_.pop();
        }
    }

    // start running this action
    private void runAction(Action a) {
        action_ = a;
        if(action_.start(actionContext_) == Action.Status.Failed) {
            context_.opMode.telemetry.log().add("failure START action {}", action_.name());
            stopAction();
        }
    }

    // stop running the current action
    private void stopAction() {
        if (action_ != null) {
            if(action_.stop(actionContext_) == Action.Status.Failed) {
                context_.opMode.telemetry.log().add("failure STOP action {}", action_.name());
            }
        }
    }
    // query the current action
    public Action CurrentAction() {
        return action_;
    }

    public void AddAction(Action a) {
        actions_.add(a);
    }
    public void AddDriver(Driver d) {
        drivers_.add(d);
    }
    public void ClearDrivers() { drivers_.clear();}
}
