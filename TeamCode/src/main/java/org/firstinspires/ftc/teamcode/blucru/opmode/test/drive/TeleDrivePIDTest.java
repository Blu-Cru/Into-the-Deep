package org.firstinspires.ftc.teamcode.blucru.opmode.test.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.PivotRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenDunkSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpecimenCycleIntakeFailsafePath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpecimenIntakeClipPath;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(group = "test")
public class TeleDrivePIDTest extends BluLinearOpMode {
    enum State{
        IDLE,
        SCANNING,
        CYCLE_INTAKE,
        CYCLE_FAILSAFE,
        CYCLE_DEPO_PATH,
        CYCLE_SCORING,
        CYCLE_DEPO_MANUAL
    }

    Path currentPath;
    Path cycleDepoPath;

    StateMachine sm;

    @Override
    public void initialize() {
        enableFTCDashboard();
        addDrivetrain();
        addPivot();
        addExtension();
        addArm();
        addClaw();
        addCVMaster();
        pivot.useExtension(extension.getMotor());
        extension.usePivot(pivot.getMotor());

        cycleDepoPath = new PIDPathBuilder().setPower(0.8)
                .schedule(new SequentialCommandGroup(
                        new ClawGrabCommand(),
                        new BoxtubeSplineCommand(
                                new Vector2d(20,42),
                                new Pose2d(-8.6, 30, Math.PI),
                                0,
                                0.95
                        )
                ))
                .addMappedPoint(7, -40, 270, 5)
                .build();

        currentPath = new SpecimenIntakeClipPath().start();

        sm = new StateMachineBuilder()
                .state(State.IDLE)
                .onEnter(() -> {
                    currentPath.cancel();
                    new FullRetractCommand().schedule();
                })
                .loop(() -> {
                    dt.teleOpDrive(gamepad1);
//                    dt.updateAprilTags(cvMaster.tagDetector);
                })
                .transition(() -> stickyG1.b && cvMaster.numDetections > 0, State.CYCLE_INTAKE, () -> {
                    dt.updateAprilTags(cvMaster.tagDetector);
                    currentPath = new SpecimenIntakeClipPath().start();
                })

                .state(State.CYCLE_INTAKE)
                .transition(() -> stickyG1.a, State.IDLE)
                .transition(() -> currentPath.isDone(), State.CYCLE_FAILSAFE, () -> currentPath = new SpecimenCycleIntakeFailsafePath().start())
//                .transition(() -> Robot.getInstance().intakeSwitch.justPressed()
//                        && Robot.getInstance().pivot.getAngle() < 0.6, State.CYCLE_DEPO_PATH, () -> currentPath = cycleDepoPath.start())
                .loop(() -> {
                    currentPath.run();
//                    dt.updateAprilTags(cvMaster.tagDetector);
                })

                .state(State.CYCLE_DEPO_PATH)
                .transition(() -> stickyG1.a, State.IDLE)
                .transition(() -> currentPath.isDone(), State.CYCLE_DEPO_MANUAL, () -> {
                    dt.setDrivePower(0.7);
                    dt.idle();
                })
                .loop(() -> currentPath.run())

                .state(State.CYCLE_DEPO_MANUAL)
                .transition(() -> stickyG1.a, State.IDLE)
                .transition(() -> stickyG1.b, State.CYCLE_SCORING, () -> {
                    dt.pidTo(dt.pose);
                    new SequentialCommandGroup(
                            new SpecimenDunkSplineCommand(),
                            new WaitCommand(280),
                            new ClawOpenCommand(),
                            new WaitCommand(150),
                            new PivotRetractCommand(),
                            new ExtensionRetractCommand(),
                            new WaitCommand(100),
                            new FullRetractCommand()
                    ).schedule();
                })
                .loop(() -> dt.pidYHeadingMapped(gamepad1.left_stick_x, -34 - gamepad1.left_stick_y * 3, -Math.PI/2))

                .state(State.CYCLE_SCORING)
                .transitionTimed(0.25, State.CYCLE_INTAKE, () -> {
                    currentPath = new SpecimenIntakeClipPath().start();
                })

                .state(State.CYCLE_FAILSAFE)
//                .transition(() -> Robot.getInstance().intakeSwitch.justPressed(), State.CYCLE_DEPO_PATH, () -> {
//                    currentPath = cycleDepoPath.start();
//                })
                .transition(() -> currentPath.isDone(), State.CYCLE_INTAKE, () -> {
                    currentPath = new SpecimenIntakeClipPath().start();
                })
                .loop(() -> currentPath.run())

                .build();
    }

    @Override
    public void onStart() {
        sm.start();
        sm.setState(State.IDLE);
        cvMaster.detectTag();
    }

    @Override
    public void periodic() {
        sm.update();
        dt.drawPose();

        if(stickyG1.right_stick_button) {
            dt.setHeading(Math.PI/2);
        }
    }

    @Override
    public void telemetry() {
        telemetry.addData("State", sm.getState());
    }
}
