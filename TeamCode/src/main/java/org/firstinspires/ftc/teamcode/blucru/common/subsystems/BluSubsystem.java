package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface BluSubsystem {

        void init();
        void read();
        void write();
        void telemetry(Telemetry telemetry);
}
