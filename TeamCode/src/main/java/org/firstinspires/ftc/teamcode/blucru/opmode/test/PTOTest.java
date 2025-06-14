package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(group = "test")
public class PTOTest extends BluLinearOpMode {
    @Override
    public void initialize() {
        addPTOServos();
        addDrivetrain();
        addHangServos();
        addExtension();
        addPivot();

        pivot.useExtension(extension.getMotor());
        extension.usePivot(pivot.getMotor());

        dt.fieldCentric = false;
    }

    @Override
    public void periodic() {
        if(stickyG1.dpad_up) {
            ptoServos.disengage();
        }

        if(stickyG1.dpad_down) {
            ptoServos.engage();
        }

        if(stickyG1.y) {
            slideHangServos.release();
        }

        if(stickyG1.a) {
            slideHangServos.hang();
        }

        dt.drive(new Pose2d(gamepad1.left_stick_x,0, 0));
    }
}
