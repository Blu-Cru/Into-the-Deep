package org.firstinspires.ftc.teamcode.blucru.opmode.test.kinematics;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.BoxtubeSpline;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Config
@TeleOp(group = "test")
public class SplineGraphTest extends BluLinearOpMode {
    public static double
        xI = 20, yI = 5, xF = 20, yF = 20, angleI = 0, angleF = 1.4,
        vX = 0, vY = 0,
        wristI = -1.57, wristF = 1.57,
        duration = 3;

    BoxtubeSpline spline;

    @Override
    public void initialize() {
        addWrist();
        addArm();
        createNewSpline();
    }

    @Override
    public void periodic() {
        if(stickyG1.a) {
            createNewSpline();
        }

        wrist.setAngle(wristI);

        spline.update();
    }

    @Override
    public void telemetry() {
//        spline.telemetry();
        spline.testTelemetry();
    }

    public void createNewSpline() {
        spline = new BoxtubeSpline(
                new Pose2d(xI, yI, angleI),
                new Vector2d(vX, vY),
                new Pose2d(xF, yF, angleF),
                wristF,
                duration
        );
    }
}
