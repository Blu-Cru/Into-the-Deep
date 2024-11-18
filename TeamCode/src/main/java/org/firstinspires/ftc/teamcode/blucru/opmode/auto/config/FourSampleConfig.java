package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;

public class FourSampleConfig extends AutoConfig {
    Path liftingPath, depositingPath,
        rightIntakePath, centerIntakePath, leftIntakePath,
        rightFailsafePath, centerFailsafePath, leftFailsafePath;

    Path currentPath;
    StateMachine sm;

    enum State{
        LIFTING,
        DEPOSITING,
        RIGHT_INTAKE,
        CENTER_INTAKE,
        LEFT_INTAKE,
        RIGHT_FAILSAFE,
        CENTER_FAILSAFE,
        LEFT_FAILSAFE,
        PARKING,
        DONE
    }

    ElapsedTime runtime;

    public FourSampleConfig() {
        runtime = Globals.runtime;

        sm = new StateMachineBuilder()
                .build();
    }

    @Override
    public void build() {

    }

    @Override
    public void start() {

    }

    @Override
    public void run() {

    }

    @Override
    public void telemetry() {

    }
}
