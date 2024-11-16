package org.firstinspires.ftc.teamcode.blucru.opmode.teleop;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

import java.util.HashMap;

public class Solo extends BluLinearOpMode {
    enum State {
        RETRACTED,
        EXTENDED_OVER_INTAKE,
        INTAKING,
        RETRACTING_FROM_INTAKE,
        LIFTING_TO_BASKET,
        AT_BASKET,
        SCORING_BASKET,
        LIFTING_TO_SPECIMEN,
        AT_SPECIMEN,
        DUNKING_SPECIMEN,
        RETRACTING_FROM_SCORING
    }

    HashMap<State, Double> drivePowers;

    State state;

    @Override
    public void initialize() {
        setDrivePowers();

        addDrivetrain();
        addExtension();
        addPivot();
        addArm();
    }

    @Override
    public void periodic() {
        updateDrivePower();
        dt.teleOpDrive(gamepad1); // driving


    }

    public void setDrivePowers() {
        drivePowers = new HashMap<State, Double>() {{
            put(State.RETRACTED, 0.9);
            put(State.EXTENDED_OVER_INTAKE, 0.7);
            put(State.INTAKING, 0.6);
            put(State.RETRACTING_FROM_INTAKE, 0.8);
            put(State.LIFTING_TO_BASKET, 0.6);
            put(State.AT_BASKET, 0.4);
            put(State.SCORING_BASKET, 0.3);
            put(State.LIFTING_TO_SPECIMEN, 0.6);
            put(State.AT_SPECIMEN, 0.4);
            put(State.DUNKING_SPECIMEN, 0.4);
            put(State.RETRACTING_FROM_SCORING, 0.7);
        }};
    }

    public void updateDrivePower() {
        try {
            dt.drivePower = drivePowers.get(state);
        } catch(NullPointerException e) {
            dt.drivePower = 0.5;
        }
    }

    @Override
    public void telemetry() {
        telemetry.addData("State", state);
    }
}
