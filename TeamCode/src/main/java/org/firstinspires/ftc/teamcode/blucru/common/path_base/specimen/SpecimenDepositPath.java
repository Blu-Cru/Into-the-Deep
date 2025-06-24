package org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenFrontClipUnderneathCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpecimenDepositPath extends PIDPathBuilder {
    public SpecimenDepositPath(int scoreCount) {
        super();
//        if(scoreCount == -1) {
            this.setPower(0.7)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new WaitCommand(50),
                                new SpecimenFrontClipUnderneathCommand()
                        ).schedule();
                    })
                    .addMappedPoint(14, -43.5, 110)
                    .callback(() -> {
                        new ClawOpenCommand().schedule();
                    });
//        } // else {
//            this.setPower(0.9)
//                    .schedule(new SequentialCommandGroup(
//                            new ClawGrabCommand(),
//                            new BoxtubeSplineCommand(
//                                    new Vector2d(20, 42),
//                                    new Pose2d(-8.6, 30, Math.PI),
//                                    0,
//                                    0.95
//                            )
//                    ))
//                    .addMappedPoint(10 - scoreCount * 1.2, -40, 270, 5)
//                    .setPower(0.5)
//                    .addMappedPoint(7 - scoreCount * 1.5, -34, 270)
//                    .schedule(new SequentialCommandGroup(
//                            new SpecimenDunkSplineCommand(),
//                            new WaitCommand(280),
//                            new ClawOpenCommand(),
//                            new WaitCommand(170),
//                            new PivotRetractCommand(),
//                            new ExtensionRetractCommand(),
//                            new EndEffectorRetractCommand()
//                    ))
//                    .waitMillis(170);
//        }
    }

    public SpecimenDepositPath() {
        this(-1);
    }
}