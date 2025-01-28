package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.ThreeWheelConstants;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.00297348641;
        ThreeWheelConstants.strafeTicksToInches = 0.0025575507181326887;
        ThreeWheelConstants.turnTicksToInches = .0029;
        ThreeWheelConstants.leftY = 7.6;
        ThreeWheelConstants.rightY = -7.6;
        ThreeWheelConstants.strafeX = -3.25;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "fL";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "bR";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "fR";
        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
    }
}




