package org.firstinspires.ftc.teamcode.hardware.minibot;

import org.firstinspires.ftc.teamcode.support.YieldHandler;

/**
 * Created by 28761 on 9/4/2018.
 */

public class Core {

    private YieldHandler yieldHandler;

    public Core() {

    }

    public void set_yield_handler(YieldHandler y) {
        yieldHandler = y;
    }

    public void yield(){
        yieldHandler.on_yield();
    }

}
