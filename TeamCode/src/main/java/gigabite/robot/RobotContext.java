package gigabite.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotContext {
    public OpMode opMode;
    public ElapsedTime elapsedTime;

    public RobotContext(OpMode m) {
        opMode = m;
        elapsedTime = new ElapsedTime();
    }
}
