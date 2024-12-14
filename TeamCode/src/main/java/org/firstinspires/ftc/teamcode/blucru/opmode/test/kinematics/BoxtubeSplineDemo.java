package org.firstinspires.ftc.teamcode.blucru.opmode.test.kinematics;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristUprightForwardCommand;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

import java.util.ArrayList;

@TeleOp(group = "test")
public class BoxtubeSplineDemo extends BluLinearOpMode {
    int count = 0, totalCount = 5;
    Pose2d[] poses = new Pose2d[totalCount];
    Vector2d[] vels = new Vector2d[totalCount];
    double[] wristAngles = new double[totalCount];
    double[] durations = new double[totalCount];

    @Override
    public void initialize() {
        addPivot();
        addExtension();
        extension.usePivot(pivot.getMotor());
        pivot.useExtension(extension.getMotor());
        addWrist();
        addArm();
        addClamp();

        poses = new Pose2d[] {
            new Pose2d(15, 15, Math.PI/4),
            new Pose2d(15, 15, 0),
            new Pose2d(-8, 25, 3*Math.PI/4),
            new Pose2d(8, 25, 0),
            new Pose2d(8, 25, Math.PI/2)
        };

        vels = new Vector2d[] {
                new Vector2d(0,0),
                new Vector2d(0,0),
                new Vector2d(0,0),
//                new Vector2d(10,30),
                new Vector2d(0,0),
                new Vector2d(0,0)
        };

        wristAngles = new double[] {
            0,
            -Math.PI/2,
            Math.PI/2,
            -Math.PI/2,
            -Math.PI/2
        };

        durations = new double[] {
            0.8,
            0.8,
            0.8,
            0.8,
            0.8
        };
    }

    @Override
    public void periodic() {
        if(stickyG1.dpad_up) {
            count++;

            if(count >= totalCount) {
                count = 0;
            }

            new BoxtubeSplineCommand(
                    vels[count],
                    poses[count],
                    wristAngles[count],
                    durations[count]
            ).schedule();
        }

        if(stickyG1.dpad_down) {
            count--;
            if(count < 0) {
                count = totalCount - 1;
            }
            new BoxtubeSplineCommand(
                    vels[count],
                    poses[count],
                    wristAngles[count],
                    durations[count]
            ).schedule();
        }

        if(stickyG1.a) {
            new ArmRetractCommand().schedule();
            new WristUprightForwardCommand().schedule();
            new BoxtubeRetractCommand().schedule();
        }
    }
}
