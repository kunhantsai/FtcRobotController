package org.firstinspires.ftc.teamcode.opmodes.mechBot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Robot2;
import org.firstinspires.ftc.teamcode.hardware.MechBot.ToboMech;
import org.firstinspires.ftc.teamcode.hardware.Sigma.ToboSigma;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

import java.io.IOException;

/**
 * Created by 28761 on 6/29/2019.
 */
@Disabled
@Autonomous(name = "Blue Out Power Shots", group = "MechBot")
public class AutoBlueOutPowerShots extends LinearOpMode {
    private ToboSigma.SkystoneLocation StoneLoc;

    protected static int LOG_LEVEL = Log.INFO;

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);

    public double auto_chassis_power = .4;

    @Override
    public void runOpMode() throws InterruptedException {
        log.info("RoboMech Autonomous runOpMode() starts (CPU_time = %.2f sec)", getRuntime());
        telemetry.addData("Initializing Robot", "Please Wait ...");
        telemetry.update();

        ToboMech robot = new ToboMech();
        robot.useTfod = true;
        // robot.set_simulation_mode(true);
        robot.configureLogging("ToboMech", LOG_LEVEL);
        configuration = new Configuration(hardwareMap, robot.getName()).configureLogging("Config", LOG_LEVEL);
        log.info("RoboMech Autonomous finished log configuration (CPU_time = %.2f sec)", getRuntime());

        try {
            // configure robot and reset all hardware

            robot.configure(configuration, telemetry, Robot2.ProgramType.AUTO_BLUE);
            configuration.apply();
            robot.initSetup(Robot2.ProgramType.AUTO_BLUE, ToboMech.StartPosition.OUT, configuration); // check
            robot.reset(true);
            robot.autoPara.setDoPowerShots(true);
            robot.showInitStatus(getRuntime());
            robot.showStatus(getRuntime());
        } catch (Exception E) {
            telemetry.addData("Init Failed", E.getMessage());
            handleException(E);
        }
        log.info("RoboMech Autonomous finished initialization (CPU_time = %.2f sec)", getRuntime());

        waitForStart();

        if (!robot.isSimulationMode())
            robot.initAfterStart();

        robot.runtime.reset();
        robot.runtimeAuto.reset();
        // run until the end of the match (driver presses STOP or timeout)
        if (opModeIsActive()) {
            try {
                robot.detectPosition();
                if (robot.autoPara.isDoPowerShots() && robot.tZone!= ToboMech.TargetZone.ZONE_C) {
                    robot.deliverFirstWobbleGoal();
                    if (opModeIsActive()) {
                        robot.doPowerShots();
                        if (opModeIsActive()){
                            robot.getSecondWobbleGoal();
                            robot.deliverSecondWobbleGoalAndShootBonusRings();
                        }
                    }
                } else {
                    robot.doHighGoalsAndPowerShots(3, 0, false);
                    robot.deliverFirstWobbleGoalAfterHighGoal();
                    if ((robot.runtimeAuto.seconds() < 25) && opModeIsActive()){
                        robot.getSecondWobbleGoalAfterHighGoal();
                        robot.deliverSecondWobbleGoalAndShootBonusRings();
                    }
                }
                robot.park();
            } catch (Exception E) {
                telemetry.addData("Error in event handler", E.getMessage());
                handleException(E);
                Thread.sleep(5000);
            }
        }
        try {
            robot.end();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    protected void handleException(Throwable T) {
        log.error(T.getMessage(), T);
        int linesToShow = 5;
        for (StackTraceElement line : T.getStackTrace()) {
            telemetry.log().add("%s.%s():%d", line.getClassName(), line.getMethodName(), line.getLineNumber());
            if (--linesToShow == 0) break;
        }
        telemetry.update();
    }
}
