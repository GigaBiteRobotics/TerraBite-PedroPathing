package drive;

import com.pedropathing.localization.Pose;

public class PersistentStorage {
    public static Pose currentPose = new Pose();
}
// add this at the end of auto:
// PersistentStorage.currentPose = drive.getPoseEstimate();

// add this at the beginning of teleop
// follower.setPoseEstimate(PersistentStorage.currentPose);