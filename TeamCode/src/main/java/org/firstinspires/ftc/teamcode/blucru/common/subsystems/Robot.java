package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.Extension;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.Pivot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.BoxtubeForwardKinematics;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.BoxtubeSpline;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.pose.BoxtubeIKPose;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector.Arm;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector.Clamp;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector.IntakeSwitch;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector.Wheel;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector.Wrist;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang.HangMotor;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang.HangServos;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.vision.CVMaster;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

import java.util.ArrayList;
import java.util.List;

public class Robot {
    private static Robot instance;
    HardwareMap hardwareMap; // reference to hardware

    // all subsystems
    public Drivetrain dt;
    public Arm arm;
    public Wrist wrist;
    public Clamp clamp;
    public Wheel wheel;
    public Pivot pivot;
    public Extension extension;
    public IntakeSwitch intakeSwitch;
    public CVMaster cvMaster;
    public Pusher pusher;
    public HangServos hangServos;
    public HangMotor hangMotor;

    // list of all subsystems
    ArrayList<BluSubsystem> subsystems;
    List<LynxModule> hubs;

    BoxtubeSpline spline;
    boolean followingSpline;

    public static Robot getInstance() {
        if(instance == null) {
            instance = new Robot();
        }
        return instance;
    }

    private Robot(){
        subsystems = new ArrayList<>();
    }

    public void create(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    // initializes subsystems
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        for(BluSubsystem subsystem : subsystems) {
            subsystem.init();
        }
    }

    public void read() {
        // follow spline
        if(followingSpline) {
            spline.update();
            if(spline.isFinished()) {
                followingSpline = false;
            }
        }

        for(LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        for(BluSubsystem subsystem : subsystems) {
            subsystem.read();
        }
    }

    public void write() {
        for(BluSubsystem subsystem : subsystems) {
            subsystem.write();
        }
    }

    public void followBoxtubeSpline(BoxtubeSpline spline) {
        this.spline = spline.start();
        followingSpline = true;
        arm.followBoxtubeSpline(spline);
        wrist.followBoxtubeSpline(spline);
        pivot.followBoxtubeSpline(spline);
        extension.followBoxtubeSpline(spline);
    }

    public void setIKPose(Pose2d pose) {
        double wristAngle = wrist.getAngle();
        BoxtubeIKPose ikPose = new BoxtubeIKPose(pose, wristAngle);
        arm.setIKPose(ikPose);
        pivot.setIKPose(ikPose);
        extension.setIKPose(ikPose);
    }

    public Pose2d getBoxtubePose() {
        return BoxtubeForwardKinematics.getEndEffectorPose(pivot.getAngle(), extension.getDistance(), arm.getAngle(), wrist.getAngle());
    }

//    public Pose3D getBoxtubeFieldPose() {
//        return BoxtubeForwardKinematics.getEndEffectorPose(pivot.getAngle(), extension.getDistance(), arm.getAngle(), wrist.getAngle());
//    }

    public double getVoltage() {
        double result = 13;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    public void telemetry(Telemetry telemetry) {
        for(BluSubsystem subsystem : subsystems) {
            subsystem.telemetry(telemetry);
        }
    }

    public Drivetrain addDrivetrain() {
        dt = new Drivetrain();
        subsystems.add(dt);
        return dt;
    }

    public Arm addArm() {
        arm = new Arm();
        subsystems.add(arm);
        return arm;
    }

    public Wrist addWrist() {
        wrist = new Wrist();
        subsystems.add(wrist);
        return wrist;
    }

    public Clamp addClamp() {
        clamp = new Clamp();
        subsystems.add(clamp);
        return clamp;
    }

    public Wheel addWheel() {
        wheel = new Wheel();
        subsystems.add(wheel);
        return wheel;
    }

    public Pivot addPivot() {
        pivot = new Pivot();
        subsystems.add(pivot);
        return pivot;
    }

    public Extension addExtension() {
        extension = new Extension();
        subsystems.add(extension);
        return extension;
    }

    public IntakeSwitch addIntakeSwitch() {
        intakeSwitch = new IntakeSwitch();
        subsystems.add(intakeSwitch);
        return intakeSwitch;
    }

    public CVMaster addCVMaster() {
        cvMaster = new CVMaster();
        subsystems.add(cvMaster);
        return cvMaster;
    }

    public Pusher addPusher() {
        pusher = new Pusher();
        subsystems.add(pusher);
        return pusher;
    }

    public HangServos addHangServos() {
        hangServos = new HangServos();
        subsystems.add(hangServos);
        return hangServos;
    }

    public HangMotor addHangMotor() {
        hangMotor = new HangMotor();
        subsystems.add(hangMotor);
        return hangMotor;
    }

    public boolean splineDone() {
        return spline.isFinished();
    }

    public void splineTelemetry() {
        Globals.tele.addData(spline.toString(), "");
    }

    // call this after every op mode
    public static void kill() {
        instance = null;
    }
}
