package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.turret.TurretCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Disabled
@TeleOp(name = "End effector Test", group = "test")
public class EndEffectorTest extends BluLinearOpMode {
    @Override
    public void initialize() {
        addArm();
        addWrist();
        addClaw();
    }

    @Override
    public void periodic() {
        if(stickyG1.left_bumper) {
            startIntaking();
        }

        if(stickyG1.right_bumper) {
            spitOut();
        }

        if(stickyG1.a) {
            retract();
        }

        if(stickyG1.b) {
            retract();
        }
    }

    public void startIntaking() {
//        arm.dropToGround();
//        turret.front();
//        claw.release();
//        wheel.intake();
    }

    public void stopIntaking() {
//        arm.dropToGround();
//        turret.front();
//        claw.grab();
//        wheel.stop();
    }

    public void spitOut() {
//        arm.dropToGround();
//        turret.front();
//        claw.release();
//        wheel.reverse();
    }

    public void retract() {
        new SequentialCommandGroup(
                new TurretCenterCommand(),
                new ArmRetractCommand(),
                new
        )
        arm.retract();
        turret.center();
        claw.grab();
    }
}
