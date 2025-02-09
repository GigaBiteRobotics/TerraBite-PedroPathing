package gigabite.actions;

import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.localization.Pose;


class MoveAction extends Action {

    private Pose stopPose_ = null;
    private Pose startPose_ = null;
    private Path path_ = null;

    // create a move action at x, y, angle
    // where angle is degrees.
    public MoveAction(String name, double x, double y, double angle) {
        super(name);
        stopPose_ = new Pose(x, y, Math.toRadians(angle));
    }

    public Status start(ActionContext context) {

        // construct a path from the followers current pose
        // to our target stop pose.
        startPose_ = context.follower.getPose();
        Point p0 = new Point(startPose_);
        Point p1 = new Point(stopPose_);
        BezierLine line = new BezierLine(p0, p1);
        path_ = new Path(line);
        context.follower.followPath(path_);
        return Status.Success;
    }

    // cleanup exiting the action.
    public Status stop(ActionContext context) {
        path_ = null;
        startPose_ = null;
        //todo: maybe tell follower to stop following our path?
        return Status.Success;
    }

    public Status update(ActionContext context) {

        // fail if for some reason, we are not following
        // this actions path.
        if(context.follower.getCurrentPath() != path_) {
            return Status.Failed;
        }
        if(context.follower.isBusy()) {
            return Status.Continue;
        }
        // pathing is following our path
        // and is no longer busy.
        // which means we should be at our end pose.
        // we could add a check here to see if the current pose
        // matches our expected end pose....
        return Status.Success;
    }
}