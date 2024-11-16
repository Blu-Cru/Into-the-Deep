package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

@Config
public class PinpointLocalizer implements Localizer {
    public static double xOffset = -149.4375, yOffset = 106.0;
    GoBildaPinpointDriver pinpoint;

    public PinpointLocalizer() {
        pinpoint = Globals.hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(xOffset, yOffset);

        pinpoint.resetPosAndIMU();
    }

    @Override
    public void update() {
        pinpoint.update();
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return pinpoint.getPosition();
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose) {
        pinpoint.setPosition(pose);
    }

    @Override
    public Pose2d getPoseVelocity() {
        return pinpoint.getVelocity();
    }
}
