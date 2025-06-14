package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawLooseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristRetractCommand;
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
            new SpinWristAngleCommand(-Math.PI/2).schedule();
        }

        if(stickyG1.dpad_up) {
            new SpinWristCenterCommand().schedule();
        }

        turret.setAngle(gamepad1.right_stick_x);
    }

    public void preIntake() {
        new SequentialCommandGroup(
                new ArmMotionProfileCommand(0),
                new SpinWristCenterCommand(),
                new TurretCenterCommand(),
                new UpDownWristAngleCommand(-Math.PI/2),
                new WaitCommand(100),
                new ClawOpenCommand()
        ).schedule();
    }

    public void grab() {
        new SequentialCommandGroup(
                new ArmMotionProfileCommand(-0.45),
                new UpDownWristAngleCommand(-Math.PI/2 + 0.45),
                new WaitCommand(200),
                new ClawGrabCommand(),
                new WaitCommand(120),
                new ArmMotionProfileCommand(0),
                new UpDownWristAngleCommand(-Math.PI/2)
        ).schedule();
    }

    public void spitOut() {
        new SequentialCommandGroup(
                new UpDownWristAngleCommand(-2.5),
                new WaitCommand(250),
                new ClawOpenCommand(),
                new WaitCommand(300),
                new ClawLooseCommand(),
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
