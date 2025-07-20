package org.firstinspires.ftc.teamcode.blucru.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.DriveBase;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Autonomous(name = "Auto", group = "1")
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

                if(opModeIsActive()) requestOpModeStop(); // stop bc not config yet

                configTelemetry();
                telemetry.addLine("RIGHT JOYSTICK BUTTON TO CONTINUE");
            })
            .transition(() -> stickyG1.right_stick_button || stickyG2.right_stick_button,
                    State.INITIALIZED, () -> {
                // just picked auto type
                gamepad1.rumble(200);
                gamepad2.rumble(200);
                telemetry.update();

                telemetry.addLine("Building Paths . . .");
                telemetry.update();

                // create autoconfig
                config = AutoConfig.config();
                        assert config != null;
                        config.build();

                telemetry.addLine("Config Built!!!! good job you deserve a pat on the back");
                telemetry.update();

                /*
                    TODO: make an intialize method in
                    each auto config to initialize differently,
                    abstract away from auto opmode
                */

//                cvMaster.detectTag();
                new ClawGrabCommand().schedule();
                new ArmCommand(1.8).schedule();
                new SpinWristAngleCommand(Math.PI).schedule();
                ptoServos.disengage();
            })
            .state(State.INITIALIZED)
            .loop(() -> {
                configTelemetry();

                telemetry.addLine("CONFIG DONE, LEFT JOYSTICK BUTTON TO CONFIG");
//                cvMaster.telemetry(telemetry);
            })
            .transition(() -> stickyG1.left_stick_button || stickyG2.left_stick_button,
                    State.CONFIG, () -> {
                gamepad1.rumble(200);
                gamepad2.rumble(200);
            })
            .transition(this::opModeIsActive, State.RUNNING, () -> {
                dt.setPoseEstimate(config.getStartPose());
                config.start();
            })
            .state(State.RUNNING)
            .loop(() -> config.run()) // runs whole auto

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
        addArm();
        addTurret();
        addUpDownWrist();
        addSpinWrist();
        addClaw();
        addPivot();
        addExtension();
        addCactus();
//        addPusher();
        addPTOServos();
//        addClapServos();
        addCVMaster();

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
        DriveBase.startPose = dt.pose;
    }

    @Override
    public void telemetry() {
        telemetry.addData("Auto state", sm.getState());
        try {
            config.telemetry();
        } catch(Exception ignored){}
    }

//    @Override
//    public void end() {
//        if(Globals.alliance == Alliance.RED) DriveBase.startPose = dt.pose;
//        else DriveBase.startPose = new Pose2d(dt.pose.vec(), Angle.norm(dt.pose.getHeading() + Math.PI));
//        Log.i("Auto", "start pose set to" + DriveBase.startPose);
//    }

    public void configTelemetry() {
        telemetry.addData("b to switch Alliance", Globals.alliance);
        telemetry.addData("a to switch Auto Type", AutoConfig.currentAutoType);
    }
}
