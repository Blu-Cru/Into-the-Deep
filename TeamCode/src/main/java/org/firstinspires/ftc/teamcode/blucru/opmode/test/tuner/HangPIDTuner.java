package org.firstinspires.ftc.teamcode.blucru.opmode.test.tuner;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Config
@TeleOp(group = "test")
public class HangPIDTuner extends BluLinearOpMode {
    public static double targetInches = 0;
    boolean hanging = false;

    @Override
    public void initialize() {
        enableFTCDashboard();
        addPTODrivetrain();
        addPTOServos();
        addHangServos();
        addExtension();
        addPivot();

        pivot.useExtension(extension.getMotor());
        extension.usePivot(pivot.getMotor());
    }

    @Override
    public void onStart() {
        slideHangServos.release();
        pivot.pidTo(0.75);
        extension.pidTo(0);
    }

    @Override
    public void periodic() {
        ptoDt.updatePID();

        if(hanging) {
            if(stickyG1.y) {
                hanging = false;
                slideHangServos.release();
                ptoServos.disengage();
            }

            if(!(gamepad1.right_trigger > 0.2)) {
                ptoDt.stopPID();
            } else if(gamepad1.a) {
                ptoDt.pidTo(targetInches);
            }
        } else {
            if (stickyG1.y) {
                hanging = true;
                slideHangServos.hang();
                ptoServos.engage();
            }

            ptoDt.teleOpDrive(gamepad1);

            if(stickyG1.right_stick_button) {
                ptoDt.setHeading(Math.PI/2);
            }
        }

    }

    @Override
    public void telemetry() {
        telemetry.addData("Target Inches", targetInches);
    }
}
