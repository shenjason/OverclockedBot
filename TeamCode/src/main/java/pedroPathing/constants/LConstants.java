package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = .001989436789;
        TwoWheelConstants.strafeTicksToInches = .001989436789;

        TwoWheelConstants.forwardY = 1;
        TwoWheelConstants.strafeX = -2.5;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "leftFront";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "rightFront";
        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
    }
}




