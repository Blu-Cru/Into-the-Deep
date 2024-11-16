package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.Extension;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.Pivot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector.Arm;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector.Clamp;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector.Wheel;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector.Wrist;

import java.util.ArrayList;

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

    // list of all subsystems
    ArrayList<BluSubsystem> subsystems;

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
//        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
//            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }

        for(BluSubsystem subsystem : subsystems) {
            subsystem.init();
        }
    }

    public void read() {
        // clear bulk cache for bulk reading
//        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
//            module.clearBulkCache();
//        }

        for(BluSubsystem subsystem : subsystems) {
            subsystem.read();
        }
    }

    public void write() {
        for(BluSubsystem subsystem : subsystems) {
            subsystem.write();
        }
    }

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
        dt = new Drivetrain(hardwareMap);
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

    // call this after every op mode
    public static void kill() {
        instance = null;
    }
}
