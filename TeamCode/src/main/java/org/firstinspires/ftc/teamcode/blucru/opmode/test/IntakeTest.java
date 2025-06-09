package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.claw.ClawLooseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.spinwrist.SpinWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.spinwrist.SpinWristCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.spinwrist.SpinWristGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.turret.TurretCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.updownwrist.UpDownWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.updownwrist.UpDownWristRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.intake.GrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.intake.PreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.intake.SpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.Pivot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector.Claw;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(group = "test")
public class IntakeTest extends BluLinearOpMode {
    enum State{
        RETRACTED,
        PREINTAKE,
        INTAKING_SPEC,
        SCORING_SPEC,
        SCORING_BASKET,

        GRABBED_GROUND,
        SENSING_GROUND,
        GRABBED_SPEC,
        SENSING_SPEC,
    }

    StateMachine sm;
    double spinWristGlobalAngle = 0;

    @Override
    public void initialize() {
        addDrivetrain();
        addArm();
        addTurret();
        addUpDownWrist();
        addSpinWrist();
        addClaw();
        addPivot();
        addExtension();
        addCactus();
        addPusher();
        addPTOServos();

        pivot.useExtension(extension.getMotor());
        extension.usePivot(pivot.getMotor());

        dt.setDrivePower(0.7);

        sm = new StateMachineBuilder()
                .state(State.RETRACTED)
                .onEnter(() -> dt.setDrivePower(1.0))
                .transition(() -> (stickyG1.left_bumper || stickyG2.left_bumper) && pivot.getAngle() < 0.2, State.PREINTAKE, () -> {
                    new SequentialCommandGroup(
                            new TurretCenterCommand(),
                            new PreIntakeCommand(),
                            new SpinWristGlobalAngleCommand(0),
                            new WaitCommand(180),
                            new ExtensionCommand(12)
                    ).schedule();
                    spinWristGlobalAngle = 0;
                })
                .transition(() -> stickyG1.y && extension.getDistance() < 2.0, State.SCORING_BASKET, () -> {
                    new SequentialCommandGroup(
                            new ArmMotionProfileCommand(0.2),
                            new BoxtubeCommand(Math.PI/2, 24.0),
                            new UpDownWristAngleCommand(1.5),
                            new SpinWristAngleCommand(Math.PI),
                            new TurretCenterCommand()
                    ).schedule();
//                    new SequentialCommandGroup(
//                            new ClawGrabCommand(),
//                            new SpinWristCenterCommand(),
//                            new ArmMotionProfileCommand(Math.PI/2),
//                            new BoxtubeCommand(Math.PI/2, 10.0),
//                            new WaitCommand(500),
//                            new TurretBackwardsCommand(),
//                            new WaitCommand(500),
//                            new ArmMotionProfileCommand(2.8),
//                            new UpDownWristAngleCommand(-2.6)
//                    ).schedule();
                })
                .transition(() -> stickyG2.x, State.INTAKING_SPEC, () -> {
                    new SequentialCommandGroup(
                            new PivotCommand(1.6),
                            new ArmMotionProfileCommand(2.67),
                            new UpDownWristAngleCommand(-1.4),
                            new ClawOpenCommand(),
                            new SpinWristAngleCommand(Math.PI),
                            new TurretCenterCommand()
                    ).schedule();
                })
                .loop(() -> {
                    if(stickyG1.right_bumper) new SpitCommand().schedule();
                })

                .state(State.PREINTAKE)
                .onEnter(() -> {
                    spinWrist.setTurretGlobalAngle(spinWristGlobalAngle);
                    dt.setDrivePower(0.45);
                })
                .transition(() -> stickyG1.a || stickyG2.a, State.RETRACTED, () -> {
                    new FullRetractCommand().schedule();
                })
                .transition(() -> stickyG1.left_bumper || stickyG2.left_bumper, State.GRABBED_GROUND, () -> {
                    new GrabCommand().schedule();
                })

                .loop(() -> {
                    double rightInput = Math.max(gamepad1.right_trigger, gamepad2.right_trigger);
                    double leftInput = Math.max(gamepad1.left_trigger, gamepad2.left_trigger);
                    if(rightInput > 0.1) {
                        turret.setAngle(-rightInput);
                    } else if (leftInput > 0.1){
                        turret.setAngle(leftInput);
                    } else {
                        turret.center();
                    }

                    if(stickyG1.dpad_up || stickyG2.dpad_up) {
                        setSpinWristGlobalAngle(0);
                    } else if (stickyG1.dpad_right || stickyG2.dpad_right) {
                        setSpinWristGlobalAngle(Math.PI/2);
                    }
                })

                .state(State.GRABBED_GROUND)
                .transitionTimed(0.63, State.SENSING_GROUND)
                .state(State.SENSING_GROUND)
                .transition(() -> cactus.validSample, State.RETRACTED, () -> {
                    new FullRetractCommand().schedule();
                })
                .transition(() -> cactus.isEmpty(), State.PREINTAKE, () -> {
                    new PreIntakeCommand().schedule();
                })
                .transitionTimed(0.15, State.PREINTAKE, () -> {
                    new PreIntakeCommand().schedule();
                })

                .state(State.SCORING_BASKET)
                .onEnter(() -> {
                    dt.setDrivePower(0.55);
                })
                .transition(() -> stickyG1.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new ClawOpenCommand(),
                            new ArmMotionProfileCommand(0),
                            new WaitCommand(50),
                            new UpDownWristAngleCommand(-1),
                            new WaitCommand(100),
                            new UpDownWristAngleCommand(-1),
                            new WaitCommand(150),
                            new ArmRetractCommand(),
                            new BoxtubeRetractCommand(),
                            new ClawLooseCommand(),
                            new SpinWristCenterCommand(),
                            new TurretCenterCommand(),
                            new UpDownWristRetractCommand()
                    ).schedule();
                })

                .state(State.INTAKING_SPEC)
                .onEnter(() -> dt.setDrivePower(0.6))
                .transition(() -> stickyG2.a, State.RETRACTED, () -> new FullRetractCommand().schedule())
                .transition(() -> (stickyG2.left_bumper || cactus.validSample) && pivot.getAngle() > 1.3, State.GRABBED_SPEC, () -> {
                    new SequentialCommandGroup(
                            new ClawGrabCommand(),
                            new WaitCommand(120),
                            new PivotCommand(1.15),
                            new UpDownWristAngleCommand(-2.0)
                    ).schedule();
                })

                .state(State.GRABBED_SPEC)
                .transitionTimed(0.3, State.SENSING_SPEC)
                .state(State.SENSING_SPEC)
                .transition(() -> cactus.isEmpty(), State.INTAKING_SPEC, () -> {
                    new SequentialCommandGroup(
                            new ClawOpenCommand(),
                            new UpDownWristAngleCommand(-1.4),
                            new WaitCommand(100),
                            new PivotCommand(1.6)
                    ).schedule();
                })
                .transitionTimed(0.1, State.INTAKING_SPEC, () -> {
                    new SequentialCommandGroup(
                            new ClawOpenCommand(),
                            new UpDownWristAngleCommand(-1.4),
                            new WaitCommand(100),
                            new PivotCommand(1.6)
                    ).schedule();
                })
                .transition(() -> cactus.validSample, State.SCORING_SPEC, () -> {
                    new SequentialCommandGroup(
                            new PivotCommand(0.63),
                            new ExtensionCommand(5.0),
                            new ArmMotionProfileCommand(0),
                            new WaitCommand(250),
                            new ExtensionCommand(10.0),
                            new UpDownWristAngleCommand(0.5),
                            new SpinWristCenterCommand(),
                            new WaitCommand(250),
                            new ClawLooseCommand()
                    ).schedule();
                })

                .state(State.SCORING_SPEC)
                .onEnter(() -> dt.setDrivePower(0.8))
                .transition(() -> stickyG2.left_bumper, State.INTAKING_SPEC, () -> {
                    new SequentialCommandGroup(
                            new ClawOpenCommand(),
                            new WaitCommand(200),
                            new ExtensionRetractCommand(),
                            new ArmMotionProfileCommand(2.67),
                            new UpDownWristAngleCommand(-1.4),
                            new ClawOpenCommand(),
                            new SpinWristAngleCommand(Math.PI),
                            new TurretCenterCommand(),
                            new WaitCommand(100),
                            new PivotCommand(1.6)
                    ).schedule();
                })
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new ClawOpenCommand(),
                            new WaitCommand(200),
                            new FullRetractCommand()
                    ).schedule();
                })

                .build();

        sm.setState(State.RETRACTED);
        sm.start();
    }

    @Override
    public void periodic() {
        if (gamepad1.getGamepadId() == -1) dt.teleOpDrive(gamepad2);
        else dt.teleOpDrive(gamepad1);

        if(gamepad1.right_stick_button || gamepad2.right_stick_button) {
            dt.setHeading(Math.PI/2);
            gamepad1.rumble(150);
        }
        sm.update();
    }

    public void setSpinWristGlobalAngle(double angle) {
        spinWristGlobalAngle = angle;
        spinWrist.setTurretGlobalAngle(angle);
    }

    @Override
    public void telemetry() {
        telemetry.addData("State", sm.getState());
        telemetry.addData("Spin wrist angle", spinWristGlobalAngle);
    }
}
