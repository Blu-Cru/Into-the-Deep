package org.firstinspires.ftc.teamcode.blucru.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.StickyGamepad;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.Extension;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.Pivot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector.Arm;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector.Clamp;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector.Wheel;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector.Wrist;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.Drivetrain;

public abstract class BluLinearOpMode extends LinearOpMode {
    public Alliance alliance;
    public Robot robot;
    public Drivetrain dt;
    public Arm arm;
    public Wrist wrist;
    public Clamp clamp;
    public Pivot pivot;
    public Wheel wheel;
    public Extension extension;

    public StickyGamepad stickyG1;
    public StickyGamepad stickyG2;

    ElapsedTime runtime;
    double lastTime;
    double loopTimeSum;
    int loopTimeCount;

    public final void runOpMode() throws InterruptedException {
        Globals.runtime = new ElapsedTime();
        Globals.hwMap = hardwareMap;
        Globals.tele = telemetry;
        CommandScheduler.getInstance().cancelAll();
        alliance = Globals.alliance;

        stickyG1 = new StickyGamepad(gamepad1);
        stickyG2 = new StickyGamepad(gamepad2);

        robot = Robot.getInstance();
        robot.create(hardwareMap);
        Globals.setVoltage(robot.getVoltage());

        initialize();
        robot.init();

        while(opModeInInit()) {
            stickyG1.update();
            stickyG2.update();
            CommandScheduler.getInstance().run();
            initLoop();
            try {
                telemetry();
            } catch (Exception e) {}
            telemetry.update();
        }
        waitForStart();
        robot.read();
        onStart();
        Globals.runtime.reset();
        runtime = new ElapsedTime(); // start timer
        runtime.reset();

        while (!isStopRequested() && opModeIsActive()) {
            stickyG1.update();
            stickyG2.update();

            // safety for switching controllers
            if(gamepad1.start || gamepad2.start) {
                continue;
            }

            robot.read();

            periodic();
            CommandScheduler.getInstance().run();
            robot.write();

            // calculate average loop time
            loopTimeSum += runtime.milliseconds() - lastTime;
            lastTime = runtime.milliseconds();
            loopTimeCount++;

            telemetry();
            robot.telemetry(telemetry);
            telemetry.addData("alliance:", alliance);
            telemetry.addData("Loop Time", loopTimeSum / loopTimeCount);
            resetLoopTime();
            telemetry.update();
        }

        end();
        Robot.kill();
        Log.i("BluLinearOpMode", "OpMode Stopped");
    }

    // methods to be overriden
    public void initialize() {}
    public void initLoop() {}
    public void onStart() {}
    public void periodic() {}
    public void telemetry() {}
    public void end() {}

    public void addDrivetrain() {
        dt = robot.addDrivetrain();
    }
    public void addArm() {arm = robot.addArm();}
    public void addWrist() {wrist = robot.addWrist();}
    public void addClamp() {clamp = robot.addClamp();}
    public void addPivot() {pivot = robot.addPivot();}
    public void addWheel() {wheel = robot.addWheel();}
    public void addExtension() {extension = robot.addExtension();}

    // enable the FTC Dashboard telemetry and field overlay
    public void enableFTCDashboard() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
    }

    public double currentSecs() {return runtime.seconds();}

    public double currentTime() {return runtime.milliseconds();}

    public double secsSince(double timeSecs) {
        return runtime.seconds() - timeSecs;
    }

    public double timeSince(double timeMillis) {
        return runtime.milliseconds() - timeMillis;
    }

    private void resetLoopTime() {
        loopTimeSum = 0;
        loopTimeCount = 0;
    }
}
