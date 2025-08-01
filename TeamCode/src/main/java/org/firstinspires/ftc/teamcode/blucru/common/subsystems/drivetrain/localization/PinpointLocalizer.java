package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

@Config
public class PinpointLocalizer implements Localizer {
    public static double parallelYOffset = -144.675, perpXOffset = -70;
    GoBildaPinpointDriver pinpoint;

    public PinpointLocalizer() {
        pinpoint = Globals.hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(parallelYOffset, perpXOffset);

        pinpoint.resetPosAndIMU();
    }

    @Override
    public void update() {
        pinpoint.update();
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        Pose2d pose = pinpoint.getPosition();
        
        // null protection
        if (Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return new Pose2d();
        }
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

    public void setHeading(double heading) {
        heading = Angle.norm(heading);
        setPoseEstimate(new Pose2d(getPoseEstimate().vec(), heading));
    }

    public double getHeading() {
        return getPoseEstimate().getHeading();
    }

    public double getHeadingVelocity() {
        return getPoseVelocity().getHeading();
    }
}
