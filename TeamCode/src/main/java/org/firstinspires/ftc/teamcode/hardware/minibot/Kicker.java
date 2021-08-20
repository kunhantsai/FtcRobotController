package org.firstinspires.ftc.teamcode.hardware.minibot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Kicker {

    Servo kicker;
    final Core core;
    final static double KICKER_INIT = 0.558;
    final static double KICKER_UP = 0.46;
    final static double KICKER_DOWN = 0.71;

    public Kicker(Core c) {
        core = c;
    }

    public void init(HardwareMap hwMap) {
        kicker = hwMap.servo.get("sv_l_kicker"); // should be kicker, not sv_l_kicker

        kicker.setPosition(KICKER_INIT);
    }

    /***
     * moves kicker up
     */
    public void kicker_up() {
        kicker.setPosition(KICKER_UP);
    }

    /***
     * moves kicker down
     */
    public void kicker_down() {
        kicker.setPosition(KICKER_DOWN);
    }
}
