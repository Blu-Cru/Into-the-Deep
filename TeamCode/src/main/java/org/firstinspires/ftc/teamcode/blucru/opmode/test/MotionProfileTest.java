package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.util.MotionProfile;

@Disabled
@Config
@TeleOp(name = "Motion Profile Test", group = "test")
public class MotionProfileTest extends LinearOpMode {
    public static double MAX_VEL = 1.0, MAX_ACCEL = 1.0,
        xI = 0.0, xF = 1.0;
    public static boolean run = false;

    MotionProfile profile;
    boolean lastRun = run;
    @Override
    public void runOpMode() throws InterruptedException {
        profile = new MotionProfile(0,0,0,0);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        waitForStart();

        while(opModeIsActive()) {
            if(run != lastRun) {
                profile = new MotionProfile(xF, xI, MAX_VEL, MAX_ACCEL).start();
            }
            lastRun = run;
            profile.getInstantState();

            profile.telemetry(telemetry);
            telemetry.update();
        }
    }
}
