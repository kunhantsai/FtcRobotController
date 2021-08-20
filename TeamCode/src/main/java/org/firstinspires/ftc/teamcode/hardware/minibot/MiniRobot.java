package org.firstinspires.ftc.teamcode.hardware.minibot;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class MiniRobot {
    public Chassis chassis;
    public Kicker kicker;
    public Pusher pusher;
    public Core core;

    public MiniRobot() {
        core = new Core();
        chassis = new Chassis(core);
        // kicker = new Kicker(core);
        // pusher = new Pusher(core);

    }

    public void init(HardwareMap hwMap) {
        chassis.init(hwMap);
    }
}
