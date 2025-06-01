package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.claw.ClawLooseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.spinwrist.SpinWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.spinwrist.SpinWristCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.turret.TurretCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.updownwrist.UpDownWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.updownwrist.UpDownWristRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "End effector Test", group = "test")
public class EndEffectorTest extends BluLinearOpMode {
    @Override
    public void initialize() {
        addArm();
        addTurret();
        addUpDownWrist();
        addSpinWrist();
        addClaw();
    }

    @Override
    public void periodic() {
        if(stickyG1.left_bumper) {
            grab();
        }

        if(stickyG1.right_bumper) {
            spitOut();
        }

        if(stickyG1.a) {
            retract();
        }

        if(stickyG1.b) {
            preIntake();
        }

        if(stickyG1.dpad_right) {
            new SpinWristAngleCommand(Math.PI/2).schedule();
        }

        if(stickyG1.dpad_up) {
            new SpinWristCenterCommand().schedule();
        }
    }

    public void preIntake() {
        new SequentialCommandGroup(
                new ArmMotionProfileCommand(0),
                new SpinWristCenterCommand(),
                new ClawOpenCommand(),
                new TurretCenterCommand(),
                new UpDownWristAngleCommand(-Math.PI/2)
        ).schedule();
    }

    public void grab() {
        new SequentialCommandGroup(
                new ArmMotionProfileCommand(-0.4),
                new UpDownWristAngleCommand(-Math.PI/2 + 0.4),
                new WaitCommand(250),
                new ClawGrabCommand(),
                new WaitCommand(250),
                new ArmMotionProfileCommand(0),
                new UpDownWristAngleCommand(-Math.PI/2)
        ).schedule();
    }

    public void spitOut() {
        new SequentialCommandGroup(
                new UpDownWristAngleCommand(-2.5),
                new WaitCommand(70),
                new ClawOpenCommand(),
                new WaitCommand(100),
                new ClawGrabCommand(),
                new UpDownWristRetractCommand()
        ).schedule();
    }

    public void retract() {
        new SequentialCommandGroup(
                new TurretCenterCommand(),
                new ArmRetractCommand(),
                new ClawLooseCommand(),
                new SpinWristCenterCommand(),
                new UpDownWristRetractCommand()
        ).schedule();
    }
}
