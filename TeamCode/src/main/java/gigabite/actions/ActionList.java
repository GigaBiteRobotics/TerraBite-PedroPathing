package gigabite.actions;

import java.util.ArrayList;
import java.util.Iterator;

public class ActionList extends Action {
    // holds our list of actions and the current action.
    private ArrayList<Action> actions_ = null;
    private int actionIndex_ = 0;
    private Action current_ = null;

    // constructor
    public ActionList(String name) {
        super(name);
        actions_ = new ArrayList<>();
    }

    // action functions

    // begin at the 1st action in the list
    public Status start(ActionContext context) {
        super.start(context);
        // start the 1st action
        actionIndex_ = 0;
        return next(context);
    }

    public Status stop(ActionContext context) {
        super.stop(context);
        current_ = null;
        actionIndex_ = 0;
        return Status.Success;
    }

    public Status update(ActionContext context) {
        super.update(context);
        context.robot.Context().opMode.telemetry.addData("ActionList", "Index: %d, Count: %d", actionIndex_, actions_.size());
        Status s = Status.Success;
        // update the current action, or go to next action
        context.robot.Context().opMode.telemetry.log().add("update: %s", current_.name());
        s = current_.update(context);
        if (s == Status.Continue) {
            return s;
        }
        s = next(context);
        return s;
    }

    // action list functions
    private Status next(ActionContext context) {
        context.robot.Context().opMode.telemetry.log().add("next: %d", actionIndex_);
        if (current_ != null) {
            context.robot.Context().opMode.telemetry.log().add("stop: %s", current_.name());
            current_.stop(context);
            current_ = null;
            // ignore failures on stop
            // and continue to next action
        }
        if (actionIndex_ < actions_.size()) {
            current_ = actions_.get(actionIndex_++);
            current_.start(context);
            context.robot.Context().opMode.telemetry.log().add("start: %s", current_.name());
            return Status.Continue;
        }
        // if there are no more actions,
        // we are done.
        return Status.Success;
    }

    public ActionList add(Action a) {
        actions_.add(a);
        return this;
    }

    public Action pop() {
        Action a = null;
        if(!actions_.isEmpty()) {
            a = actions_.get(0);
            actions_.remove(0);
        }
        return a;
    }

}