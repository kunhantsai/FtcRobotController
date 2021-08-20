package org.firstinspires.ftc.teamcode.hardware.minibot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Pusher {

    Servo pusher;
    final Core core;

    final static double PUSHER_INIT = 0.63;
    final static double PUSHER_UP = 0.78;
    final static double PUSHER_DOWN = 0.44;

    public Pusher(Core c) {
        core = c;
    }

    public void init(HardwareMap hwMap) {
        pusher = hwMap.servo.get("sv_r_kicker"); // should be pusher, not sv_r_pusher

        pusher.setPosition(PUSHER_INIT);
    }

    /***
     * moves pusher up
     */
    public void pusher_up() {
        pusher.setPosition(PUSHER_UP);
    }

    /***
     * moves pusher down
     */
    public void pusher_down() {
        pusher.setPosition(PUSHER_DOWN);
    }
}
