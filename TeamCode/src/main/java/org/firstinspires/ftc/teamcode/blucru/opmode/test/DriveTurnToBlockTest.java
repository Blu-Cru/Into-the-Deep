package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Config
@TeleOp(name="Drive Turn To Block Test", group="test")
public class DriveTurnToBlockTest extends BluLinearOpMode {
    public static double blockX = 0, blockY = 0;

    enum State {
        TELEOP,
        TURN_TO_BLOCK
    }

    StateMachine sm;

    @Override
    public void initialize() {
        addDrivetrain();
        enableFTCDashboard();

        sm = new StateMachineBuilder()
                .state(State.TELEOP)
                .transition(() -> stickyG1.a, State.TURN_TO_BLOCK, () -> {
                    gamepad1.rumble(300);
                })
                .loop(() -> {
                    dt.teleOpDrive(gamepad1);
                    if(gamepad1.right_stick_button) {
                        dt.setHeading(Math.PI/2);
                    }
                })
                .state(State.TURN_TO_BLOCK)
                .transition(() -> stickyG1.a, State.TELEOP, () -> {
                    gamepad1.rumble(300);
                })
                .loop(() -> {
                    dt.teleDriveTurnToPos(gamepad1.left_stick_x, -gamepad1.left_stick_y, new Vector2d(blockX, blockY));
                })
                .build();
    }

    @Override
    public void periodic() {
        sm.update();
        dt.drawPose();
    }

    @Override
    public void telemetry() {
        telemetry.addData("State", sm.getState());
    }
}
