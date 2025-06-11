package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants {
    static {
        TwoWheelConstants.forwardEncoder_HardwareMapName = "rightFront";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "leftFront";
        TwoWheelConstants.forwardY = 1;
        TwoWheelConstants.strafeX = 1;
        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.IMU_HardwareMapName = "imu"; // Name in your hardware map

        // Optional: Set IMU orientation if needed
        // TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(
        //     RevHubOrientationOnRobot.LogoFacingDirection.UP,
        //     RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        // );
    }
}