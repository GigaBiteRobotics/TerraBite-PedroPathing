package gigabite.actions;

import java.util.ArrayList;

// action that will run multiple actions in parallel
// will continue if any actions continue
public class OrAction extends Action{
    private ArrayList<Action> actions_ = null;

    public OrAction(String name) {
        super(name);
    }

    public Status start(ActionContext context) {
        for( Action a : actions_) {
            Status s = a.start(context);
            if (s == Status.Failed) {
                return s;
            }
        }
        return Status.Success;
    }
    public Status stop(ActionContext context) {
        for( Action a : actions_) {
            Status s = a.stop(context);
            if (s == Status.Failed) {
                return s;
            }
        }
        return Status.Success;
    }

    public Status update(ActionContext context) {
        // in order to continue, only one must continue
        Status rv = Status.Success;
        for( Action a : actions_) {
            Status s = a.stop(context);
            if (s == Status.Continue) {
                rv = Status.Continue;
            }
        }
        return rv;
    }
}
