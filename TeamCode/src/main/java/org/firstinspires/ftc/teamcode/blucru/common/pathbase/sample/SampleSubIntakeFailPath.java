package org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelReverseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SampleSubIntakeFailPath extends PIDPathBuilder {
    public SampleSubIntakeFailPath() {
        super();
        this.callback(() -> {
                    new BoxtubeSplineCommand(
                            new Pose2d(8, 8, -0.65),
                            -Math.PI/2,
                            0.4
                    ).schedule();
                    new SequentialCommandGroup(
                            new WaitCommand(120),
                            new WheelReverseCommand(),
                            new WaitCommand(100),
                            new ClampGrabCommand(),
                            new WheelStopCommand()
                    ).schedule();
                })
                .addMappedPoint(-26, -12, 0)
                .waitMillis(400);
    }
}
