package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.Extension;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.Pivot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.BoxtubeForwardKinematics;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.BoxtubeSpline;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.pose.BoxtubeIKPose;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.PTODrivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization.OrthogonalDistanceSensors;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.end_effector.Arm;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.end_effector.CactusSensor;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.end_effector.Claw;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.end_effector.DualCactusSensor;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.end_effector.SpinWrist;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.end_effector.Turret;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.end_effector.UpDownWrist;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang.HangMotor;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang.slide_hang_servo.SlideHangServos;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang.pto_servo.PTOServos;
import org.firstinspires.ftc.teamcode.blucru.common.vision.CVMaster;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point3d;

import java.util.ArrayList;
import java.util.List;

public class Robot {
    private static Robot instance;
    HardwareMap hardwareMap; // reference to hardware

    // all subsystems
    public Drivetrain dt;
    public PTODrivetrain ptoDt;
    public Arm arm;
    public Turret turret;
    public Claw claw;
    public SpinWrist spinWrist;
    public UpDownWrist upDownWrist;
    public Pivot pivot;
    public Extension extension;
    public CVMaster cvMaster;
    public Pusher pusher;
    public SlideHangServos slideHangServos;
    public HangMotor hangMotor;
    public DualCactusSensor cactus;
    public PTOServos ptoServos;
    public OrthogonalDistanceSensors orthogonalDistanceSensors;

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

    public static boolean validSample() {
        return getInstance().cactus.validSample();
    }


    public static boolean justValidSample() {
        return getInstance().cactus.justValidSample();
    }

    public void followBoxtubeSpline(BoxtubeSpline spline) {
        this.spline = spline.start();
        followingSpline = true;
        arm.followBoxtubeSpline(spline);
        turret.followBoxtubeSpline(spline);
        pivot.followBoxtubeSpline(spline);
        extension.followBoxtubeSpline(spline);
    }

    public void setIKPose(Pose2d pose) {
        double wristAngle = turret.getAngle();
        BoxtubeIKPose ikPose = new BoxtubeIKPose(pose, wristAngle);
        arm.setIKPose(ikPose);
        pivot.setIKPose(ikPose);
        extension.setIKPose(ikPose);
    }

    public Pose2d getBoxtubePose() {
        return BoxtubeForwardKinematics.getEndEffectorPose(pivot.getAngle(), extension.getDistance(), arm.getAngle(), turret.getAngle());
    }

    public Point3d getBoxtubePoint3d() {
        Pose2d pose = dt.pose;
        double heading = dt.heading;
        Pose2d boxtubePose = BoxtubeForwardKinematics.getEndEffectorPose(pivot.getAngle(), extension.getDistance(), arm.getAngle(), turret.getAngle());

        double x = pose.getX() + boxtubePose.getX() * Math.cos(heading);
        double y = pose.getY() + boxtubePose.getX() * Math.sin(heading);
        double z = pose.getY();

        return new Point3d(x, y, z);
    }

    public double getVoltage() {
        double result = 12;
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

    public Turret addTurret() {
        turret = new Turret();
        subsystems.add(turret);
        return turret;
    }

    public Claw addClaw() {
        claw = new Claw();
        subsystems.add(claw);
        return claw;
    }

    public SpinWrist addSpinWrist() {
        spinWrist = new SpinWrist();
        subsystems.add(spinWrist);
        return spinWrist;
    }

    public UpDownWrist addUpDownWrist() {
        upDownWrist = new UpDownWrist();
        subsystems.add(upDownWrist);
        return upDownWrist;
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

    public SlideHangServos addHangServos() {
        slideHangServos = new SlideHangServos();
        subsystems.add(slideHangServos);
        return slideHangServos;
    }

    public HangMotor addHangMotor() {
        hangMotor = new HangMotor();
        subsystems.add(hangMotor);
        return hangMotor;
    }

    public DualCactusSensor addCactus() {
        cactus = new DualCactusSensor();
        subsystems.add(cactus);
        return cactus;
    }

    public PTOServos addPTOServos() {
        ptoServos = new PTOServos();
        subsystems.add(ptoServos);
        return ptoServos;
    }

    public PTODrivetrain addPTODrivetrain() {
        ptoDt = new PTODrivetrain();
        subsystems.add(ptoDt);
        return ptoDt;
    }
    public OrthogonalDistanceSensors addOrthogonalDistanceSensors() {
        orthogonalDistanceSensors = new OrthogonalDistanceSensors();
        subsystems.add(orthogonalDistanceSensors);
        return orthogonalDistanceSensors;
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
