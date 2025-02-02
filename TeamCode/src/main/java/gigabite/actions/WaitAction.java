package gigabite.actions;

// an action that waits for a specific amount of time
public class WaitAction extends Action{
private final double msToWait_;

public WaitAction(String name, double ms) {
    super(name);
    msToWait_ = ms;
}

public Status start(ActionContext context) {
    return Status.Success;

}
public Status stop( ActionContext context) {
    return Status.Success;
}

public Status update(ActionContext context) {
    if( milliseconds() < msToWait_) {
        return Status.Continue;
    }
    return Status.Success;
}

}

