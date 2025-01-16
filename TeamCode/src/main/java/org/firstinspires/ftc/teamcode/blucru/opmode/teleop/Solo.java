package org.firstinspires.ftc.teamcode.blucru.opmode.teleop;

import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

public class Solo extends BluLinearOpMode {
    enum State {
        RETRACTED,
        EXTENDING_OVER_INTAKE,
        INTAKING_GROUND,
        SCORING_BASKET,
        INTAKING_SPECIMEN,
        ABOVE_SPECIMEN_FRONT,
        ABOVE_SPECIMEN_BACK,
        DUNKING_SPECIMEN_FRONT,
        DUNKING_SPECIMEN_BACK,
        RETRACTING_FROM_SCORING,
        RETRACTING_FROM_INTAKE,
        MANUAL_RESET,

        AUTO_BASKET,
        AUTO_TO_ASCENT,
        AUTO_TO_RUNG,
        AUTO_SPECIMEN_INTAKE
    }

    StateMachine sm;
    Path currentPath;

    @Override
    public void initialize() {
        addDrivetrain();
        addExtension();
        addPivot();
        addArm();
        addWheel();
        addClamp();
        addWrist();
        addIntakeSwitch();
        addPusher();
        addHangServos();
        extension.usePivot(pivot.getMotor());
        pivot.useExtension(extension.getMotor());
    }
}
