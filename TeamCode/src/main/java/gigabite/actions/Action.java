package gigabite.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Action {

    private ElapsedTime timer_ = null;
    private String name_ = null;

    public enum Status {
        Success,
        Failed,
        Continue
    }

    public Action(String name) {
        name_ = name;
        timer_ = new ElapsedTime();
    }

    // the time spent in this action
    public ElapsedTime Timer() {
        return timer_;
    }

    public double milliseconds() {
        return timer_.milliseconds();
    }
    public double seconds() {
        return timer_.time();
    }

    public String name() {
        return name_;
    }

    public Status start(ActionContext context) {
        timer_.reset();
        return Status.Success;
    }
    public Status stop(ActionContext context) {
        return Status.Success;
    }

    // call update() as long as it returns Continue
    public Status update(ActionContext context) {
        return Status.Success;
    }
}


