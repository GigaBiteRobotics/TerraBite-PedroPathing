package gigabite.actions;

import java.util.ArrayList;

// action that will run multiple actions in parallel
// will continue only if all actions continue
public class AndAction extends Action{
    protected ArrayList<Action> actions_ = null;

    public AndAction(String name) {
        super(name);
        actions_ = new ArrayList<>();
    }

    public Status start(ActionContext context) {
        super.start(context);
        for( Action a : actions_) {
            Status s = a.start(context);
            if (s == Status.Failed) {
                return s;
            }
        }
        return Status.Success;
    }
    public Status stop(ActionContext context) {
        super.stop(context);
        for( Action a : actions_) {
            Status s = a.stop(context);
            if (s == Status.Failed) {
                return s;
            }
        }
        return Status.Success;
    }

    public Status update(ActionContext context) {
        super.update(context);
        // in order to continue, all must continue.
        for( Action a : actions_) {
            Status s = a.update(context);
            if (s != Status.Continue) {
                return s;
            }
        }
        return Status.Continue;
    }
}
