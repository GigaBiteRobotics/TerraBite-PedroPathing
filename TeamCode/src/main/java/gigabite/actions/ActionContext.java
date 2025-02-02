package gigabite.actions;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;


// stores the global data and accessors
// for running actions
public interface ActionContext {
    Follower follower();
    ElapsedTime elapsedTime();
}
