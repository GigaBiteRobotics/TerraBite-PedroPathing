package gigabite.actions;

import java.util.ArrayList;
import java.util.Iterator;

public class ActionList extends Action {
    // holds our list of actions and the current action.
    private ArrayList<Action> actions_ = null;
    private Iterator<Action> actionIt_ = null;
    private Action current_ = null;

    // constructor
    public ActionList(String name) {
        super(name);
        actions_ = new ArrayList<>();
    }

    // action functions

    // begin at the 1st action in the list
    public Status start(ActionContext context) {
        // start the 1st action
        actionIt_ = actions_.iterator();
        return next(context);
    }

    public Status stop(ActionContext context) {
        current_ = null;
        actionIt_ = null;
        return Status.Success;
    }

    public Status update(ActionContext context) {
        Status s = Status.Success;
        // update the current action
        // and go to next action
        if(current_ != null) {
            s = current_.update(context);
            if (s == Status.Continue) {
                return s;
            }
            s = next(context);
        }
        return s;
    }

    // action list functions
    private Status next(ActionContext context) {
        Status s = Status.Success;
        if (current_ != null) {
            current_.stop(context);
            current_ = null;
            // ignore failures on stop
            // and continue to next action
        }
        if (actionIt_.hasNext()) {
            current_ = actionIt_.next();
            if (current_ != null) {
                s = current_.start(context);
            }
        }
        return s;
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