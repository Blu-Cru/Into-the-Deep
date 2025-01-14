package org.firstinspires.ftc.teamcode.blucru.common.path;

import android.telecom.Call;

public class CallbackTest implements Callback {
    public void run() {
        Callback callback = () -> {
            System.out.println("Hello, world!");
        };
    }
}
