package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.GetHooksCommand;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(group = "test")
public class PTOTest extends BluLinearOpMode {
    @Override
    public void initialize() {
        addPTOServos();
        addPTODrivetrain();
        addHangServos();
        addClapServos();
        addArm();
        addClaw();
        addUpDownWrist();
        addSpinWrist();
        addTurret();
        addExtension();
        addPivot();

        pivot.useExtension(extension.getMotor());
        extension.usePivot(pivot.getMotor());

        ptoDt.fieldCentric = false;
    }

    @Override
    public void periodic() {
        if(stickyG1.dpad_up) {
            ptoServos.disengage();
        }

        if(stickyG1.dpad_down) {
            ptoServos.engage();
        }

        if(stickyG1.dpad_left) {
            slideHangServos.release();
        }

        if(stickyG1.dpad_right) {
            slideHangServos.hang();
        }

        if(stickyG1.b) {
            new GetHooksCommand().schedule();
        }

        if(stickyG1.a) {
            new FullRetractCommand().schedule();
        }

        ptoDt.drive(new Pose2d(gamepad1.left_stick_x,0, 0));
    }
}
