package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmDropToGroundCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenFrontCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenFrontDunkCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenFrontDunkRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen.SpecimenPreloadDepositPath;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "April Tag Path Test", group = "test")
public class AprilTagPathTest extends BluLinearOpMode {
    enum State {
        DRIVER_CONTROL,
        FOLLOWING_PATH,
    }
    State state = State.DRIVER_CONTROL;
    StateMachine sm;
    Path path;

    @Override
    public void initialize() {
        addDrivetrain();
        addCVMaster();
        enableFTCDashboard();

        sm = new StateMachineBuilder()
                .state(State.DRIVER_CONTROL)
                .onEnter(() -> {
                    dt.idle();
                })
                .transition(() -> stickyG1.a, State.FOLLOWING_PATH, () -> {
                    path = new PIDPathBuilder().setPower(0.7)
                            .addMappedPoint(7, -40, 90, 6)
                            .setPower(0.3)
                            .addMappedPoint(7, -33, 90)
                            .waitMillis(600)
                            .addMappedPoint(20, -44, 45, 2)
                            .addMappedPoint(28, -36, 23,3)
                            .waitMillis(1000).build().start();
                })
                .loop(() -> {
                    dt.teleOpDrive(gamepad1);

                    if(stickyG1.right_stick_button) {
                        dt.setHeading(Math.PI/2);
                    }
                })
                .state(State.FOLLOWING_PATH)
                .transition(() -> stickyG1.a, State.DRIVER_CONTROL)
                .transition(() -> path.isDone(), State.DRIVER_CONTROL)
                .loop(() -> {
                    path.run();
                })
                .build();
    }

    @Override
    public void onStart() {
        sm.start();
        sm.setState(State.DRIVER_CONTROL);
        cvMaster.detectTag();
    }

    @Override
    public void periodic() {
        sm.update();
        dt.updateAprilTags(cvMaster.tagDetector);
        dt.drawPose();
    }

    @Override
    public void telemetry() {
        telemetry.addData("State", state);
    }
}
