package org.firstinspires.ftc.teamcode.blucru.opmode.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

public class Auto extends BluLinearOpMode {
    enum State {
        CONFIG,
        INITIALIZED,
        RUNNING,
        RESETTING
    }

    AutoConfig config;

    StateMachine sm = new StateMachineBuilder()
            .state(State.CONFIG)
            .loop(() -> {
                if(stickyG1.b || stickyG2.b) Globals.flipAlliance();
                if(stickyG1.a || stickyG2.a) AutoConfig.flipAutoType();

                if(opModeIsActive()) requestOpModeStop();

                configTelemetry();
                telemetry.addLine("RIGHT JOYSTICK BUTTON TO CONTINUE");
            })
            .transition(() -> stickyG1.right_stick_button || stickyG2.right_stick_button,
                    State.INITIALIZED, () -> {
                gamepad1.rumble(200);
                gamepad2.rumble(200);
                telemetry.update();

                telemetry.addLine("Building Paths . . .");
                telemetry.update();
                config = AutoConfig.config();
                config.build();

                config.setStartPose();
                telemetry.addLine("Config Built!!!! good job you deserve a pat on the back");
                telemetry.update();

                // initialize pivot position
                new SequentialCommandGroup(
                        new PivotCommand(0.7),
                        new WaitCommand(800),
                        new ArmGlobalAngleCommand(-0.5)
                ).schedule();
            })
            .state(State.INITIALIZED)
            .loop(() -> {
                configTelemetry();

                telemetry.addLine("LEFT JOYSTICK BUTTON TO CONFIG");
            })
            .transition(() -> stickyG1.left_stick_button || stickyG2.left_stick_button,
                    State.CONFIG, () -> {
                gamepad1.rumble(200);
                gamepad2.rumble(200);
            })
            .transition(this::opModeIsActive, State.RUNNING, () -> {
                dt.setPoseEstimate(Globals.startPose);
                config.start();
            })
            .state(State.RUNNING)
            .loop(() -> {
                config.run(); // runs whole auto
            })
            .transition(() -> gamepad1.share, State.RESETTING, () -> {
                config.stop();
                new FullRetractCommand().schedule();
            })
            .state(State.RESETTING)
            .loop(() -> {
                dt.teleOpDrive(gamepad1);
            })
            .build();

    @Override
    public void initialize() {
        addDrivetrain();
        addExtension();
        addPivot();
        addArm();
        addWheel();
        addWrist();
        addClamp();
        addIntakeSwitch();
        extension.usePivot(pivot.getMotor());
        pivot.useExtension(extension.getMotor());

        sm.setState(State.CONFIG);
        sm.start();

        enableFTCDashboard();
    }

    @Override
    public void initLoop() {
        robot.read();
        sm.update();
        robot.write();
    }

    public void periodic() {
        sm.update();
    }

    @Override
    public void telemetry() {
        telemetry.addData("Auto state", sm.getState());
        config.telemetry();
    }

    public void configTelemetry() {
        telemetry.addData("b to switch Alliance", Globals.alliance);
        telemetry.addData("a to switch Auto Type", AutoConfig.autoType);
    }
}
