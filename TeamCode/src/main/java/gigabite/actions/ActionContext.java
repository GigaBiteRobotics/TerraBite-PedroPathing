package gigabite.actions;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;

import gigabite.robot.Robot;


// stores the global data and accessors
// for running actions
public class ActionContext {
    public Follower follower;
    public Robot robot;
    public ElapsedTime elapsedTime;
}
