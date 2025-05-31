package org.firstinspires.ftc.teamcode.blucru.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.StickyGamepad;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Pusher;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.Extension;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.Pivot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector.Arm;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector.CactusSensor;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector.Clamp;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector.IntakeSwitch;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector.Wheel;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector.Turret;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang.HangMotor;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang.HangServos;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.vision.CVMaster;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public abstract class BluLinearOpMode extends LinearOpMode {
    public Robot robot;
    public Drivetrain dt;
    public Arm arm;
    public Turret turret;
    public Clamp clamp;
    public Pivot pivot;
    public Wheel wheel;
    public Extension extension;
    public IntakeSwitch intakeSwitch;
    public Pusher pusher;
    public HangServos hangServos;
    public HangMotor hangMotor;
    public CVMaster cvMaster;
    public CactusSensor cactus;

    public StickyGamepad stickyG1, stickyG2;

    double lastTime, loopTimeSum, loopTimeAvg = 0;
    int loopTimeCount;

    public final void runOpMode() throws InterruptedException {
        Globals.runtime = new ElapsedTime();
        Globals.hwMap = hardwareMap;
        Globals.tele = telemetry;
        CommandScheduler.getInstance().cancelAll();

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

            // safety for switching controllers
            if(gamepad1.start || gamepad2.start) {
                continue;
            }

            initLoop();
            CommandScheduler.getInstance().run();
            telemetry();
            telemetry.update();
        }
        waitForStart();
        Globals.runtime = new ElapsedTime();
        robot.read();
        onStart();

        while (opModeIsActive()) {
            stickyG1.update();
            stickyG2.update();

            // safety for switching controllers
            if(!(gamepad1.start || gamepad2.start)) {
                periodic();
            }

            CommandScheduler.getInstance().run();
            robot.read();
            robot.write();

            telemetry();
            robot.telemetry(telemetry);
            telemetry.addData("Alliance:", Globals.alliance);
            telemetry.addData("Loop Time", calculateAvgLoopTime());
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

    public void addDrivetrain() {dt = robot.addDrivetrain();}
    public void addArm() {arm = robot.addArm();}
    public void addWrist() {
        turret = robot.addWrist();}
    public void addClamp() {clamp = robot.addClamp();}
    public void addPivot() {pivot = robot.addPivot();}
    public void addWheel() {wheel = robot.addWheel();}
    public void addExtension() {extension = robot.addExtension();}
    public void addIntakeSwitch() {intakeSwitch = robot.addIntakeSwitch();}
    public void addCVMaster() {cvMaster = robot.addCVMaster();}
    public void addPusher() {pusher = robot.addPusher();}
    public void addHangServos() {hangServos = robot.addHangServos();}
    public void addHangMotor() {hangMotor = robot.addHangMotor();}
    public void addCactus() {cactus = robot.addCactus();}

    // enable the FTC Dashboard telemetry and field overlay
    public void enableFTCDashboard() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        Globals.tele = telemetry;
    }

    private double calculateAvgLoopTime() {
        loopTimeSum += Globals.runtime.milliseconds() - lastTime;
        lastTime = Globals.runtime.milliseconds();
        loopTimeCount++;

        if(loopTimeCount > 5) {
            loopTimeAvg = loopTimeSum / loopTimeCount;
            loopTimeSum = 0;
            loopTimeCount = 0;
        }

        return loopTimeAvg;
    }
}
