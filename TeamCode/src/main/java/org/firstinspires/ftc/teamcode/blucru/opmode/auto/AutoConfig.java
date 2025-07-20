package org.firstinspires.ftc.teamcode.blucru.opmode.auto;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.config.FiveSpecimenConfig;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.config.FiveSpecimenOneSampleConfig;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.config.FourSampleConfig;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.config.FourSpecimenOneSampleConfig;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.config.SampleCycleConfig;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.config.SixSpecimenConfig;

// abstract class for auto config
public abstract class AutoConfig {
    static AutoType currentAutoType = AutoType.FOUR_SAMPLE;

    /*
    TODO: create new class for autotype, create a better flip method
        Also move auto config to common
     */
    enum AutoType{
        FOUR_SAMPLE,
        FIVE_SPECIMEN,
        FIVE_SPECIMEN_ONE_SAMPLE,
        FOUR_SPECIMEN_ONE_SAMPLE,
        SAMPLE_CYCLE,
        SIX_SPECIMEN;

        public AutoType flip() {
            if(this == FOUR_SAMPLE) return FIVE_SPECIMEN;
            else if (this == FIVE_SPECIMEN) return FIVE_SPECIMEN_ONE_SAMPLE;
            else if (this == FIVE_SPECIMEN_ONE_SAMPLE) return SAMPLE_CYCLE;
            else if (this == SAMPLE_CYCLE) return FOUR_SPECIMEN_ONE_SAMPLE;
            else if (this == FOUR_SPECIMEN_ONE_SAMPLE) return SIX_SPECIMEN;
            else return FOUR_SAMPLE;
        }
    }

    public StateMachine sm;
    public Path currentPath;
    public ElapsedTime runtime;

    // abstract methods to be implemented by the auto config
    // TODO: create initialize here
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
    // TODO: organize this better
    public static AutoConfig config() {
        switch (currentAutoType) {
            case FOUR_SAMPLE: return new FourSampleConfig();
            case FIVE_SPECIMEN: return new FiveSpecimenConfig();
            case FIVE_SPECIMEN_ONE_SAMPLE: return new FiveSpecimenOneSampleConfig();
            case SAMPLE_CYCLE: return new SampleCycleConfig();
            case FOUR_SPECIMEN_ONE_SAMPLE: return new FourSpecimenOneSampleConfig();
            case SIX_SPECIMEN: return new SixSpecimenConfig();
        }
        return null;
    }

    public static void flipAutoType() {
        currentAutoType = currentAutoType.flip();
    }

    public void logTransition(Enum state) {
        Log.i("Auto Config", " transitioning to " + state);
    }
}
