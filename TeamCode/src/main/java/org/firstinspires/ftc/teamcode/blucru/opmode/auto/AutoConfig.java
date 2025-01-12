package org.firstinspires.ftc.teamcode.blucru.opmode.auto;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.config.FiveSpecimenConfig;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.config.FourSampleConfig;

// abstract class for auto config
public abstract class AutoConfig {
    static AutoType autoType = AutoType.FOUR_SAMPLE;

    enum AutoType{
        FOUR_SAMPLE,
//        FIVE_SAMPLE,
//        FOUR_SPECIMEN,
        FIVE_SPECIMEN;

        public AutoType flip() {
            if(this == FOUR_SAMPLE) {
                return FIVE_SPECIMEN;
            } else {
                return FOUR_SAMPLE;
//            } else if(this == FOUR_SPECIMEN) {
//                return FIVE_SPECIMEN;
//            } else {
//                return FOUR_SPECIMEN;
            }
        }
    }

    public StateMachine sm;
    public Path currentPath;
    public ElapsedTime runtime;

    // abstract methods to be implemented by the auto config
    public abstract void build();
    public abstract void start();
    public abstract void telemetry();
    public abstract Pose2d getStartPose();

    public void run() {
        sm.update();
        currentPath.run();
    }

    public void stop() {
        currentPath.cancel();
    }

    // this method returns the correct auto config based on the current side and auto type
    public static AutoConfig config() {
        switch (autoType) {
            case FOUR_SAMPLE:
                return new FourSampleConfig();
//            case FIVE_SAMPLE:
//                return new FiveSampleConfig();
//            case FOUR_SPECIMEN:
//                return new FourSpecimenConfig();
            case FIVE_SPECIMEN:
                return new FiveSpecimenConfig();
        }
        return null;
    }

    public static AutoType getAutoType() {
        return autoType;
    }
    public static void flipAutoType() {
        autoType = autoType.flip();
    }

    public void logTransition(Enum state) {
        Log.i("Auto Config", " transitioning to " + state);
    }
}
