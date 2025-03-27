package gigabite.actions;

// an action that waits for a specific amount of time
public class WaitAction extends Action{
private final double seconds_;

public WaitAction(String name, double seconds) {
    super(name);
    seconds_ = seconds;
}

public Status start(ActionContext context) {
    return Status.Success;

}
public Status stop( ActionContext context) {
    return Status.Success;
}

public Status update(ActionContext context) {
    super.update(context);
    if( seconds() < seconds_) {
        return Status.Continue;
    }
    return Status.Success;
}

}

