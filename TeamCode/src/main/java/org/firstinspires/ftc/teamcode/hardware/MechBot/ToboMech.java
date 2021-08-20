package org.firstinspires.ftc.teamcode.hardware.MechBot;

import android.util.Log;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.components.MechChassis;
import org.firstinspires.ftc.teamcode.components.Robot2;
import org.firstinspires.ftc.teamcode.components.SwerveChassis;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.diagnostics.MenuEntry;
import org.firstinspires.ftc.teamcode.support.events.Button;
import org.firstinspires.ftc.teamcode.support.events.EventManager;
import org.firstinspires.ftc.teamcode.support.events.Events;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.tasks.Progress;
import org.firstinspires.ftc.teamcode.support.tasks.Task;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.lang.annotation.Target;

import static android.util.Log.DEBUG;
import static java.lang.Thread.interrupted;
import static java.lang.Thread.sleep;

public class ToboMech extends Logger<ToboMech> implements Robot2 {

    public class AutoPara {
        boolean doPowerShots = false;
        boolean isDone = false;

        public boolean isDone() {
            return isDone;
        }
        public boolean isDoPowerShots() {
            return doPowerShots;
        }

        public void setDoPowerShots(boolean doPowerShots) {
            this.doPowerShots = doPowerShots;
        }
    }

    public enum TargetZone {
        ZONE_A, ZONE_B, ZONE_C, UNKNOWN
    }

    public enum StartPosition {
        IN, OUT, NA;
    }

    public final double MIN_STICK_VAL = 0.1;
    public TargetZone tZone = TargetZone.UNKNOWN;
    public ProgramType side = ProgramType.AUTO_BLUE; // default to blue
    public StartPosition startPos = StartPosition.OUT; // default to OUT position
    public AutoPara autoPara = null;

    Thread positionThread;
    private Telemetry telemetry;
    private Configuration cfg;
    public MechChassis chassis;
    public CoreSystem core;
    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime runtimeAuto = new ElapsedTime();
    private ElapsedTime EventTimer = new ElapsedTime();
    private double waitSec;
    public double rotateRatio = 0.7; // slow down ratio for rotation
    public CameraDetector cameraDetector;
    public File simEventFile;
    public ComboGrabber comboGrabber;
    public Shooter shooter;
    public Hopper hopper;
    public Intake intake;

    private double cycleTime = 0; // nano-sec
    public double auto_chassis_power = .5;
    public double auto_chassis_dist = 200;
    public double auto_chassis_heading = 0;
    public double auto_chassis_power_slow = .4;
    public double auto_chassis_align_power = .22;
    public double shooter_offset = 14; // shooter is 10 cm right of the robot center x coordination
    public double webcam_offset_x = 25; // webcam is 25 cm left of the robot center x coordination
    public double webcam_offset_y = -14.2;
    public double shooting_dist = 0;
    public double shooting_angle = 0;
    public double shooterAngleOffset = 2.5;
    final public double MAX_RPM = 1720;
    final public double WARM_UP_RPM = 1660;
    static final public double WARM_UP_RPM_AUTO = 1600;
    final public double SEMI_AUTO_RPM = 1660;
    final public double WARM_UP_RPM_POWER_SHOT = 1440;
    final public double SEMI_POWER_SHOT_RPM = 1440;
    final public double SEMI_HIGHGOAL_HEADING = -12; // 0 for tile-2, -9 for tile-3
    public double shooting_rpm = WARM_UP_RPM;
    public double batteryVolt = 0;

    public double auto_rotate_degree = 0;

    private boolean simulation_mode = false;
    private boolean useChassis = true;
    public boolean useVuforia = false;
    public boolean useTfod = false;
    public boolean useComboGrabber = true;
    public boolean useHopper = true;
    public boolean useShooter = true;
    public boolean useIntake = true;
    public boolean isTeleOpAfterAuto = false;
    public boolean targetHighGoal = true; // when false, it target power shot
    private boolean useIMUforOdometryAngleCorrection = true; // use IMU for radian correction

    public void updateCycleTime(long val) {
        cycleTime = val; // milli-seconds
    }
    public void set_simulation_mode(boolean value) {
        simulation_mode = value;
        if (simulation_mode) {
            if (chassis != null) {
                chassis.set_simulation_mode(value);
            }
        }
    }

    public boolean isSimulationMode() {
        return simulation_mode;
    }

    public void configureVisualTool(Configuration configuration) {
        if (!useTfod && !useVuforia) return;
        if (!simulation_mode) {
            cameraDetector = new CameraDetector();
            cameraDetector.configure(configuration, useTfod);
        }
    }

    @Override
    public String getName() {
        return getClass().getSimpleName();
    }

    @Override
    public void configure(Configuration configuration, Telemetry telemetry, ProgramType autoside) throws FileNotFoundException {
        runtime.reset();
        cfg = configuration;
        double ini_time = runtime.seconds();
        this.telemetry = telemetry;
        simEventFile = AppUtil.getInstance().getSettingsFile("ToboMech_events.txt"); // at First/settings directory
        autoPara = new ToboMech.AutoPara();

        // simFile = Paths.get("ToboMech_events.txt");
        this.core = new CoreSystem();
        info("RoboMech configure() after new CoreSystem()(run time = %.2f sec)", (runtime.seconds() - ini_time));

        if (useChassis) {
            // chassis = new MechChassis(core).configureLogging("Mecanum", logLevel); // Log.DEBUG
            chassis = new MechChassis(core).configureLogging("Mecanum", Log.DEBUG); // Log.DEBUG
            chassis.set_simulation_mode(simulation_mode);
            if (chassis != null) {
                // chassis.simOS = new FileOutputStream(new File(simEventFile.getParentFile(), simEventFile.getName()));
                chassis.simOS = new FileOutputStream(new File(simEventFile.getParent(), simEventFile.getName()));
            }
            if (autoside == ProgramType.DIAGNOSIS) {
                // enable imu for diagnosis
                // chassis.enableImuTelemetry(configuration);
            }
            chassis.configure(configuration, true, isTeleOpAfterAuto);
            // chassis.configure(configuration, (autoside != ProgramType.TELE_OP), isTeleOpAfterAuto);
        }
        if (simulation_mode) { // need to call after chassis is initialized
            set_simulation_mode(true);
        }

        if (useComboGrabber && !simulation_mode) {
            comboGrabber = new ComboGrabber(core);
            comboGrabber.configure(configuration, (autoside != ProgramType.TELE_OP));
        }

        if (useHopper && !simulation_mode) {
            hopper = new Hopper(core);
            hopper.configure(configuration, (autoside != ProgramType.TELE_OP));
        }
        if (useShooter && !simulation_mode) {
            shooter = new Shooter(core);
            shooter.configure(configuration, (autoside != ProgramType.TELE_OP));
        }
        if (useIntake && !simulation_mode) {
            intake = new Intake(core);
            intake.configure(configuration, (autoside != ProgramType.TELE_OP));
        }

        info("ToboMech configure() after init Chassis (run time = %.2f sec)", (runtime.seconds() - ini_time));
    }


    @Override
    public void reset(boolean auto) {
        if (simulation_mode == true)
            return;
        if (chassis != null)
            chassis.reset();
        if (comboGrabber != null)
            comboGrabber.servoInit();
        if (hopper != null)
            hopper.servoInit();
        if (!auto) {
            chassis.setupTelemetry(telemetry);
        }
    }

    public void end() throws InterruptedException, IOException {
        if (simulation_mode && (chassis != null)) {
            try {
                chassis.simOS.flush();
            } finally {
                chassis.simOS.close();
            }
            // ReadWriteFile.writeFile(simEventFile, chassis.getSimEvents());
            if (simulation_mode) {
                telemetry.addData("Running simulation mode and dump events to file:", "%s/%s", simEventFile.getParent(), simEventFile.getName());
                telemetry.addData("Content:", "%s", chassis.getSimEvents());
                telemetry.update();
                sleep(3000);
            }
        }
//        if (!interrupted() && (chassis!=null)) {
//            chassis.setupTelemetry(telemetry);
//            telemetry.update();
//            sleep(10000);
//        }
        if (chassis != null) {
            chassis.updateInitPosFromOdo();
            cfg.storе(); // store updated jason file
        }

        if (cameraDetector != null) {
            cameraDetector.end();
        }
    }

    @MenuEntry(label = "TeleOp", group = "Test Chassis")
    public void mainTeleOp(EventManager em) {
        setupTelemetry(telemetry);
        em.updateTelemetry(telemetry, 1000);
        if (chassis != null && chassis.getGPS() == null) {
            chassis.configureOdometry(telemetry);
            positionThread = (chassis.getGPS() == null ? null : new Thread(chassis.getGPS()));
            if (positionThread != null)
                positionThread.start();
        }
        if (chassis!=null) {
            batteryVolt = getBatteryVoltage();
            chassis.set_auto_power_scale_by_voltage(batteryVolt);
        }
        if (intake != null)
            intake.setupTelemetry(telemetry);
        if (hopper != null)
            hopper.setupTelemetry(telemetry);
        if (shooter != null)
            shooter.setupTelemetry(telemetry);
        if (comboGrabber != null)
            comboGrabber.setupTelemetry(telemetry);
        if (cameraDetector != null)
            cameraDetector.setupTelemetry(telemetry);
        em.onStick(new Events.Listener() { // Left-Joystick
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX, float currentY, float changeY) throws InterruptedException {
                if (Math.abs(source.getStick(Events.Side.RIGHT, Events.Axis.Y_ONLY)) > MIN_STICK_VAL)
                    return; // avoid conflicting drives
                if (chassis == null) return;

                double right_x = source.getStick(Events.Side.RIGHT, Events.Axis.X_ONLY);
                double normalizeRatio = chassis.getMecanumForwardRatio();

                if (!chassis.getNormalizeMode())
                    normalizeRatio = 1;

                // Left joystick for forward/backward and turn
                if (Math.abs(currentX) > MIN_STICK_VAL) {
                    double scale = chassis.powerScale()*normalizeRatio;
                    if (scale>0.6) scale=0.6; // fix turn power to be 0.6 for the odometry accuracy
                    else if (scale<0.2) scale=0.2;
                    //chassis.turn((currentX > 0 ? 1 : -1), Math.abs(currentX * currentX) * scale);
                    chassis.turn((currentX > 0 ? 1 : -1), Math.abs(scaleRotatePower(currentX))*scale);

                } else if (Math.abs(currentY) > MIN_STICK_VAL) { // car mode
                    chassis.carDrive(currentY * Math.abs(currentY) * normalizeRatio * chassis.powerScale(), right_x);
                } else if (Math.abs(currentY) > MIN_STICK_VAL) {
                    //chassis.yMove((currentY > 0 ? 1 : -1), Math.abs(currentY * currentY) * chassis.powerScale() * normalizeRatio);
                    if( chassis.powerScale()<1.0 ) { 
                       chassis.yMove((currentY > 0 ? 1 : -1), Math.abs(scaleDrivePowerLowSensitivity(currentY, normalizeRatio)));
                    } else {
                       chassis.yMove((currentY > 0 ? 1 : -1), Math.abs(scaleDrivePower(currentY, normalizeRatio)));
                    }
                } else {
                    chassis.stop();
                }
            }
        }, Events.Axis.BOTH, Events.Side.LEFT);

        em.onStick(new Events.Listener() { // Right-Joystick
            @Override
            public void stickMoved(EventManager source, Events.Side side, float currentX, float changeX, float currentY, float changeY) throws InterruptedException {
                if (chassis == null) return;
                double movingAngle = 0;
                double normalizeRatio = chassis.getMecanumForwardRatio(); // minimum 0.5 when moving forward, and maximum 1.0 when crabbing 90 degree

                if (!chassis.getNormalizeMode())
                    normalizeRatio = 1;

                if (Math.abs(source.getStick(Events.Side.LEFT, Events.Axis.Y_ONLY)) > MIN_STICK_VAL)
                    return; // avoid conflicting drives
                double left_x = source.getStick(Events.Side.LEFT, Events.Axis.X_ONLY);
                // right joystick for free crabbing
                if (Math.abs(left_x) > MIN_STICK_VAL) {
                    if (Math.abs(currentY) > MIN_STICK_VAL + 0.4) {
                        // car drive
                        chassis.carDrive(currentY * Math.abs(currentY) * normalizeRatio, left_x);
                    } else { // turn over-write free style
                        //chassis.turn((left_x > 0 ? 1 : -1), 0.6 * Math.abs(left_x * left_x) * chassis.powerScale() * normalizeRatio);
                        chassis.turn((left_x > 0 ? 1 : -1), Math.abs(scaleRotatePower(left_x)) * normalizeRatio);
                    }
                } else if (Math.abs(currentX) + Math.abs(currentY) > MIN_STICK_VAL) { // free style
                    movingAngle = Math.toDegrees(Math.atan2(currentX, currentY));

                    if (!chassis.getNormalizeMode()) {
                        normalizeRatio = 1;
                    } else if (movingAngle >= -90 && movingAngle <= 90) {
                        normalizeRatio = chassis.getMecanumForwardRatio() + (1 - chassis.getMecanumForwardRatio()) * (Math.abs(movingAngle) / 90.0);
                    } else { // movingAngle is < -90 or > 90
                        normalizeRatio = chassis.getMecanumForwardRatio() + (1 - chassis.getMecanumForwardRatio()) * ((180 - Math.abs(movingAngle)) / 90.0);
                    }
                    double lsx = Math.max(Math.abs(currentX), 1.75 * chassis.getMinPower() * normalizeRatio) * Math.signum(currentX);
                    double lsy = Math.max(Math.abs(currentY), chassis.getMinPower() * normalizeRatio) * Math.signum(currentY);
                    double power_lf = (lsy + lsx) * chassis.getFront_ratio() * chassis.getLeft_ratio();
                    double power_lb = (lsy - lsx) * chassis.getBack_ratio() * chassis.getLeft_ratio();
                    double power_rf = (lsy - lsx) * chassis.getFront_ratio() * chassis.getRight_ratio();
                    double power_rb = (lsy + lsx) * chassis.getBack_ratio() * chassis.getRight_ratio();

                    power_lf = Range.clip(power_lf, -1, 1);
                    power_lb = Range.clip(power_lb, -1, 1);
                    power_rf = Range.clip(power_rf, -1, 1);
                    power_rb = Range.clip(power_rb, -1, 1);

                    power_lf *= Math.abs(power_lf) * chassis.powerScale() * normalizeRatio;
                    power_lb *= Math.abs(power_lb) * chassis.powerScale() * normalizeRatio;
                    power_rf *= Math.abs(power_rf) * chassis.powerScale() * normalizeRatio;
                    power_rb *= Math.abs(power_rb) * chassis.powerScale() * normalizeRatio;
                    chassis.freeStyle(power_lf, power_rf, power_lb, power_rb, true);
                } else {
                    chassis.stop();
                }
            }
        }, Events.Axis.BOTH, Events.Side.RIGHT);

        em.onTrigger(new Events.Listener() {
            @Override
            public void triggerMoved(EventManager source, Events.Side side, float current, float change) throws InterruptedException {
                // 0.2 is a dead zone threshold for the trigger
                if (current > 0.2 && chassis != null && source.getTrigger(Events.Side.LEFT) < 0.2) {
                    if (source.isPressed(Button.DPAD_UP)) { // high goal
                        if (source.isPressed(Button.BACK))
                            rotateToTargetAndStartShooter(MechChassis.ShootingTarget.TOWER, false);
                        else
                            doHighGoalsAndPowerShots(3, 0, true);
                    } else if (source.isPressed(Button.DPAD_RIGHT)) { // high goal
                        rotateToTargetAndStartShooter(MechChassis.ShootingTarget.PSHOT_R, false);
                    } else if (source.isPressed(Button.DPAD_LEFT)) { // high goal
                        if (source.isPressed(Button.BACK))
                            doHighGoalsAndPowerShots(1, 2, true);
                        else
                        rotateToTargetAndStartShooter(MechChassis.ShootingTarget.PSHOT_L, false);
                    } else if (source.isPressed(Button.DPAD_DOWN)) { // high goal
                        if (source.isPressed(Button.BACK))
                            doHighGoalsAndPowerShots(2, 1, true);
                        else
                            rotateToTargetAndStartShooter(MechChassis.ShootingTarget.PSHOT_M, false);
                    }
                }
            }
        }, Events.Side.RIGHT);

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (chassis != null && source.isPressed(Button.BACK) && source.isPressed(Button.START)) {
                    // chassis.resetOdometry(false);
                    // calibration mode. Only uncomment when testing new motors with chassis wheels suspended
                    // chassis.setupEncoders(telemetry);
//                    chassis.freeStyle(1.0, 1.0, 1.0, 1.0, true);
//                    sleep(10000);
//                    chassis.stop();
                } else if (source.isPressed(Button.RIGHT_BUMPER)) {
                    if (intake != null)
                        intake.stop();
                    if (hopper != null) hopper.hopperUpCombo(false);
                } else if (source.isPressed(Button.LEFT_BUMPER)) {
                    if (shooter != null) {
                        shooter.shootSpeedInc();
                        shooting_rpm = shooter.shooterSpeed;
                    }
                } else if (source.getTrigger(Events.Side.LEFT) > 0.2 && chassis != null) { // shoot high goal using Vuforia (x,y)
                    rotateToTargetAndStartShooter(MechChassis.ShootingTarget.TOWER, false);
                }  else if(source.getTrigger(Events.Side.RIGHT) > 0.2){
                    if(hopper != null){
                        hopper.sliderUp(true);
                    }
                } else if (source.getTrigger(Events.Side.RIGHT) < 0.2 && source.getTrigger(Events.Side.LEFT) < 0.2) {
                    if (intake != null) {
//                        if (hopper != null && !hopper.getTransferIsDown())
//                            hopper.hopperDownCombo();
//                        intake.intakeInAuto();
                        if (hopper.getTransferIsDown()) {
                            intake.intakeInAuto();
                        } else {
                            hopperDownIntakeCombo();
                            TaskManager.processTasks();
                        }
                    }
                }
            }
        }, new Button[]{Button.DPAD_UP});
        em.onButtonUp(new Events.Listener() {
            @Override
            public void buttonUp(EventManager source, Button button) throws InterruptedException {

            }
        }, new Button[]{Button.DPAD_UP});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                chassis.toggleSlowMode();
            }
        }, new Button[]{Button.LEFT_STICK});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.RIGHT_BUMPER)) {
                    if (shooter != null)
                        shooter.stop();
                    hopper.hopperDownCombo();
                } else if (source.isPressed(Button.LEFT_BUMPER)) {
                    if (shooter != null) {
                        shooter.shootSpeedDec();
                        shooting_rpm = shooter.shooterSpeed;
                    }
                } else if(source.getTrigger(Events.Side.RIGHT) > 0.2){
                    if(hopper != null){
                        hopper.sliderDown(true);
                    }
                } else {
                    if (chassis != null) {
                        if (source.isPressed(Button.BACK)) {
                            // chassis.chassis_test();
                        } else if (source.getTrigger(Events.Side.RIGHT) < 0.2 && source.getTrigger(Events.Side.LEFT) < 0.2) {
                            if (intake != null)
                                intake.intakeOutAuto();
                        }
                    }
                }
            }
        }, new Button[]{Button.DPAD_DOWN});

        em.onButtonUp(new Events.Listener() {
            @Override
            public void buttonUp(EventManager source, Button button) throws InterruptedException {

            }
        }, new Button[]{Button.DPAD_DOWN});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
//                if (chassis!=null) {
//                    chassis.crab(0.45, 30, 3);
//                }
                if (source.getTrigger(Events.Side.LEFT) > 0.3) {
                    if (cameraDetector != null)
                        cameraDetector.dec_cam_pos();
                } else if (source.isPressed(Button.RIGHT_BUMPER)) {
                    // autoIntakeRings(3, false);
                    autoIntakeRingsNew(3, false);
                } else if (source.isPressed(Button.LEFT_BUMPER)) {
                    shooting_rpm = SEMI_POWER_SHOT_RPM;
                    shooter.shootOutByRpm(shooting_rpm);
                }
            }
        }, new Button[]{Button.DPAD_RIGHT});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
//                if (chassis!=null) {
//                    chassis.crab(0.45, -30, 3);
//                }
                if (source.getTrigger(Events.Side.LEFT) > 0.3) {
                    if (cameraDetector != null)
                        cameraDetector.inc_cam_pos();
                } else if (source.isPressed(Button.RIGHT_BUMPER)) {
                    if (intake != null)
                        intake.stop();
                    if (hopper != null) hopper.transferShakeCombo();
                } else if (source.isPressed(Button.LEFT_BUMPER)) {
                    shooting_rpm = WARM_UP_RPM;
                    shooter.shootOutByRpm(shooting_rpm);
                }

            }
        }, new Button[]{Button.DPAD_LEFT});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                // if (source.isPressed(Button.BACK)) {
                //    if (chassis!=null) chassis.toggleNormalizeMode();
                if (source.getTrigger(Events.Side.LEFT) > 0.3) {
                    if (source.isPressed(Button.LEFT_BUMPER)) {
                        if (comboGrabber != null)
                            comboGrabber.armDown();
                    } else {
                        if (comboGrabber != null)
                            comboGrabber.grabberAuto();
                    }
                } else if (source.getTrigger(Events.Side.RIGHT) > 0.3) {
                    if (hopper != null)
                        hopper.hopperDownCombo();
                } else if (source.isPressed(Button.LEFT_BUMPER)) {
                    if (source.isPressed(Button.BACK)) {
                        doHighGoalsAndPowerShots(3, 0, true);
                    } else {
                        if (comboGrabber != null) {
                            // comboGrabber.sliderDown(source.isPressed(Button.BACK));
                            comboGrabber.sliderPosInit();
                        }
                    }
                } else if (source.isPressed(Button.RIGHT_BUMPER)) {
                    if (comboGrabber != null)
                        comboGrabber.armDownInc();
                } else if (source.isPressed(Button.BACK)) {
                    doHighGoalsSemi(false, 3);
                } else if (!source.isPressed((Button.START))) {
                    if (hopper != null) {
                        if (hopper.getTransferIsDown() || Math.abs(shooting_rpm-WARM_UP_RPM)>20 ||
                                shooter.getCurrentRPM()<WARM_UP_RPM-100) {
                            shooting_rpm = WARM_UP_RPM;
                            shooter.shootOutByRpm(shooting_rpm);
                            hopper.hopperUpCombo(false);
                            targetHighGoal = true;
                        } else {
                            doHighGoalsSemi(false, 1);
                        }

                        // hopper.feederAuto();
                        // autoShoot();
                    }
                }
            }
        }, new Button[]{Button.A});

        em.onButtonUp(new Events.Listener() {
            @Override
            public void buttonUp(EventManager source, Button button) throws InterruptedException {
                if (comboGrabber != null)
                    comboGrabber.sliderStop();
                //if (hopper != null)
                //    hopper.transferStop();

            }
        }, new Button[]{Button.A});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {

                if (source.getTrigger(Events.Side.LEFT) > 0.3) {
                    if (source.isPressed(Button.LEFT_BUMPER)) {
                        if (comboGrabber != null)
                            comboGrabber.armUp();
                    } else {
                        if (comboGrabber != null)
                            comboGrabber.armAuto();
                    }
                } else if (source.isPressed(Button.LEFT_BUMPER)) {
                    if (source.isPressed(Button.A)) { // LB-A+Y
                        ;
                    } else {
                        if (comboGrabber != null)
                            comboGrabber.sliderUp(source.isPressed(Button.BACK));
                    }
                } else if (source.isPressed(Button.RIGHT_BUMPER)) {
                    if (comboGrabber != null)
                        comboGrabber.armUpInc();
                } else if (source.getTrigger(Events.Side.RIGHT) > 0.3) {
                    if (hopper != null)
                        hopper.transferUp();

                } else if (source.isPressed(Button.BACK)) {
                    // semi power shot
                    // doPowerShotsSemi(3,false);
                    doPowerShotsSemiNew(3,true, true);
                } else {
                    if (hopper.getTransferIsDown() || Math.abs(shooting_rpm-WARM_UP_RPM_POWER_SHOT)>20 ||
                    shooter.getCurrentRPM()<WARM_UP_RPM_POWER_SHOT-100) {
                        shooting_rpm = WARM_UP_RPM_POWER_SHOT;
                        shooter.shootOutByRpm(shooting_rpm);
                        hopper.hopperUpCombo(false);
                        targetHighGoal = false;
                    } else {
                        doPowerShotsSemi(1,false);
                    }

                }
            }
        }, new Button[]{Button.Y});
        em.onButtonUp(new Events.Listener() {
            @Override
            public void buttonUp(EventManager source, Button button) throws InterruptedException {
                if (comboGrabber != null)
                    comboGrabber.sliderStop();
                if (source.isPressed(Button.RIGHT_BUMPER)) {
                    ;
                }
                //if (hopper != null)
                //    hopper.transferStop();
            }
        }, new Button[]{Button.Y});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {

                if (source.getTrigger(Events.Side.LEFT) > 0.3) {
                    if (comboGrabber != null) {
                        comboGrabber.releaseWobbleGoalCombo();
                    }
                } else if (source.isPressed(Button.LEFT_BUMPER)) {
                    if (comboGrabber != null) {
                        autoReleaseHighWobbleGoal();
                    }
                } else if (source.isPressed(Button.BACK)) {
                    comboGrabber.releaseWobbleGoalCombo();
                } else {
                    if (hopper != null){
                        hopper.blockerAuto();
                    }
                }
            }
        }, new Button[]{Button.X});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {

                if (source.getTrigger(Events.Side.LEFT) > 0.3) {
                    if (comboGrabber != null) {
                        autoGrabBottomWobbleGoal();
                        chassis.stop();
                        // comboGrabber.grabWobbleGoalCombo(false);
                    }
                } else if (source.isPressed(Button.LEFT_BUMPER)) {
                    if (comboGrabber != null)
                        autoGrabHighWobbleGoal(true);
                    //top wobble goal combos functions go here
                } else if (source.isPressed(Button.RIGHT_BUMPER)) {
                    if (hopper != null) {
                        hopper.ringBarAuto();
                    }
                } else if (source.isPressed(Button.BACK)) {
                    if (comboGrabber != null)
                        autoGrabHighWobbleGoal(false);
                } else {
                    if (comboGrabber.isArmLow()) {
                        comboGrabber.initWobbleGoalCombo();
                    } else {
                        comboGrabber.readyGrabCombo();
                    }
                }
            }
        }, new Button[]{Button.B});

//        em.onButtonDown(new Events.Listener() {
//            @Override
//            public void buttonDown(EventManager source, Button button) throws InterruptedException {
//             shooter.shootAutoFast();
//            }
//        }, new Button[]{Button.LEFT_BUMPER});
//
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.LEFT_BUMPER)) {
                    if (shooter != null) {
                        shooter.shootAutoFast();
                    }
                }
            }
        }, new Button[]{Button.RIGHT_BUMPER});

    }
    public void showInitStatus(double init_time) throws InterruptedException {
        telemetry.addData("Robot is ready", "(init-time=%2.1f) Press Play",init_time);
        telemetry.update();
    }

    public void showStatus(double init_time) throws InterruptedException {
        String mode = "";
        if (side == ProgramType.TELE_OP) mode = "TeleOp";
        else if (side == ProgramType.DIAGNOSIS) mode = "Diagnosis";
        else if (side == ProgramType.AUTO_BLUE) {
            if (startPos == StartPosition.IN) {
                if (autoPara.doPowerShots) {
                    mode = "Blue-In-PS";
                } else {
                    mode = "Blue-In-HG";
                }
            } else {
                if (autoPara.doPowerShots) {
                    mode = "Blue-Out-PS";
                } else {
                    mode = "Blue-Out-HG";
                }
            }
        } else { // Red
            if (startPos == StartPosition.IN) {
                if (autoPara.doPowerShots) {
                    mode = "Red-In-PS";
                } else {
                    mode = "Red-In-HG";
                }
            } else {
                if (autoPara.doPowerShots) {
                    mode = "Red-Out-PS";
                } else {
                    mode = "Red-Out-HG";
                }
            }
        }
        if (cameraDetector != null) {
            sleep(2000);
            tZone = cameraDetector.getTargetZone();
        }
        telemetry.addData("Config._1", "%s | Simu.=%s | Chassis=%s",
                mode, (simulation_mode ? "Yes" : "No"), (useChassis ? "Yes" : "No"));
        telemetry.addData("Config._2", "Tensorflow=%s(%s) | Vuforia=%s",
                (useTfod ? "Yes" : "No"), tZone, (useVuforia ? "Yes" : "No"));

        telemetry.addData("Config._3", "Grabber=%s|Shooter=%s|Intake=%s|Hopper=%s",
                (useComboGrabber ? "Yes" : "No"), (useShooter ? "Yes" : "No"), (useIntake ? "Yes" : "No"), (useHopper ? "Yes" : "No"));
        if (chassis != null) {
            if (chassis.getGPS() == null) {
                telemetry.addData("Warning", "GPS is not initialized.");
            }
            if (chassis.orientationSensor == null) {
                telemetry.addData("Warning", "IMU is not initialized.");
            }
        }
        telemetry.update();
    }

    public void setupTelemetryDiagnostics(Telemetry telemetry) {
        if (chassis == null) return;
        Telemetry.Line line = telemetry.addLine();
        line.addData("Auto ", new Func<String>() {
            @Override
            public String value() {
                return String.format("Mode=%s\n", chassis.getAutoDriveMode().toString());
            }
        });
        line.addData("Test ", new Func<String>() {
            @Override
            public String value() {
                return String.format("Power/auto=%.2f/%.2f, dist=%.0f, rotate_degree=%.1f, tar-x=%.0f,tar-y=%.0f\n",
                        auto_chassis_power, chassis.auto_power, auto_chassis_dist, auto_rotate_degree,
                        chassis.auto_target_x, chassis.auto_target_y);
            }
        });
        line.addData("Result ", new Func<String>() {
            @Override
            public String value() {
                return String.format("dist_err=%.2f, degree_err=%.2f, loop_time=%3.2f, travel_p=%.3f\n",
                        chassis.auto_dist_err, chassis.auto_degree_err, chassis.auto_loop_time, chassis.auto_travel_p);
            }
        });
    }

    public void setupTelemetry(Telemetry telemetry) {
        if (Thread.currentThread().isInterrupted()) return;
        if (chassis == null) return;
        Telemetry.Line line = telemetry.addLine();
        batteryVolt = getBatteryVoltage();
        if (chassis!=null) {
            chassis.set_auto_power_scale_by_voltage(batteryVolt);
        }
        if (side==ProgramType.TELE_OP) { // TeleOp
            line.addData(" | Shooting (dist,angle,tps,volt,c-time) =", new Func<String>() {
                @Override
                public String value() {
                    return String.format("(%1.0f,%1.0f,%1.0f,%2.1f,%2.2fms)\n",
                            shooting_dist, shooting_angle, shooting_rpm, getBatteryVoltage(),cycleTime/1000000.0);
                }
            });
            if (useVuforia && (cameraDetector != null)) {
                line.addData(" | Vuforia (X,Y) =", new Func<String>() {
                @Override
                public String value() {
                    double[] vuforia_position = cameraDetector.getPositionFromVuforia();
                    return String.format("(%1.0f,%1.0f)\n", vuforia_position[0] + webcam_offset_x, vuforia_position[1] + webcam_offset_y);
                }
                });
            }
        } else { // Autonomous
            line.addData("<Y> Mode", new Func<String>() {
                @Override
                public String value() {
                    return String.format("%s\n", (autoPara.doPowerShots ? "Power-shots" : "High-goals-only"));
                }
            });
        }
        /* Telemetry.Line line = telemetry.addLine();

        line.addData(" | Odometry (vl,vr,h) =", new Func<String>() {
            @Override
            public String value() {
                return String.format("(%5d,%5d,%5d)\n", chassis.verticalLeftEncoder().getCurrentPosition(),
                        chassis.verticalRightEncoder().getCurrentPosition(),chassis.horizontalEncoder().getCurrentPosition());
            }
        });
        line.addData("Odo (x, y, angle) =", new Func<String>() {
            @Override
            public String value() {
                return String.format("(%4.0f, %4.0f, %4.0f)\n", chassis.odo_x_pos(),chassis.odo_y_pos(),chassis.odo_heading());
            }
        });

        line.addData("Offensive", new Func<String>() {
            @Override
            public String value() {
                return String.format("%s\n",  (autoPara.isOffensive() ? "Yes" : "No"));
            }
        });
        */
    }

    private double toDegrees(double x, double y) {
        if (x == 0) return y >= 0 ? 0 : 180;
        return Math.atan2(x, y) / Math.PI * 180;
    }

    public void driveThisPath() throws InterruptedException {
        MechChassis.Point[] points = {new MechChassis.Point(20, 0, 10),
                new MechChassis.Point(40, 0, 20),
                new MechChassis.Point(60, 0, 40),
                new MechChassis.Point(90, 0, 60),
                new MechChassis.Point(100, 20, 65),
                new MechChassis.Point(120, 40, 65),
                new MechChassis.Point(120, -40, 75),
                new MechChassis.Point(100, -20, 75),
                new MechChassis.Point(80, 0, 75),
                new MechChassis.Point(60, 0, -70),
                new MechChassis.Point(40, 0, -70)
        };

        //chassis.set_init_pos(30, 70, 250);
        chassis.driveThrough(.5, points, false, 10);
    }

    public void driveCurve() throws InterruptedException {
        MechChassis.Point[] points = {new MechChassis.Point(50, 50, 45),
                //new MechChassis.Point(100, 100, 90),
                //new MechChassis.Point(80, 140, 90),
                new MechChassis.Point(100, 100, 90)};

        MechChassis.Point[] points2 = {
                new MechChassis.Point(-20, 130, 0),
                new MechChassis.Point(-40, 150, 0),
                new MechChassis.Point(-20, 170, 0),
                new MechChassis.Point(0, 200, 0)};

        chassis.set_init_pos(0, 0, 0);
        //chassis.rotateTo(.5, 20);
        chassis.driveThrough(.5, points, false, 3);
        // chassis.driveThrough(.5, points2, false, 5);
        // chassis.rotateTo(.5, 90);
    }

    public void driveCircle() throws InterruptedException {
        MechChassis.Point[] points = getPointsInCircle(new MechChassis.Point(chassis.odo_x_pos_cm(), chassis.odo_y_pos_cm(), chassis.getCurHeading()),
                new MechChassis.Point(chassis.odo_x_pos_cm(), chassis.odo_y_pos_cm() + 50, 0), 1, 30);

        // chassis.set_init_pos(20, 90, 0);
        chassis.driveThrough(.6, points, false, 2);

    }

    public MechChassis.Point[] getPointsInCircle(MechChassis.Point start, MechChassis.Point center, int clockwise, double degreesPerPoint) {
        double radius = Math.hypot(start.x - center.x, start.y - center.y);

        MechChassis.Point[] p = new MechChassis.Point[(int) (360 / degreesPerPoint)];
        for (int i = 0; i < p.length; i++) {
            p[i] = new MechChassis.Point(0, 0, 0);
        }
        double initDegree = Math.atan2(start.x, start.y);
        double degree = initDegree;
        if (clockwise == 1) {
            for (int i = 0; degree < initDegree + 360; degree = degree + degreesPerPoint) {
                p[i] = new MechChassis.Point(radius * Math.sin(Math.toRadians(degree)) + center.x, radius * Math.cos(Math.toRadians(degree)) + center.y, 0);
                System.out.println(degree);
                if (i != 0) {
                    p[i - 1].h = Math.toDegrees(Math.atan2(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y));
                }                 //System.out.println(p[i].x + "  " + p[i].y);
                i++;
                System.out.println(i);
            }
        } else {
            for (int i = 0; degree > initDegree - 360; degree = degree - degreesPerPoint) {
                p[i] = new MechChassis.Point(radius * Math.sin(Math.toRadians(degree)) + center.x, radius * Math.cos(Math.toRadians(degree)) + center.y, 0);
                System.out.println(degree);
                if (i != 0) {
                    p[i - 1].h = Math.toDegrees(Math.atan2(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y));
                }
                //System.out.println(p[i].x + "  " + p[i].y);
                i++;
                System.out.println(i);
            }
        }
        p[p.length - 1] = p[0];
        p[p.length - 2].h = Math.toDegrees(Math.atan2(p[p.length - 1].x - p[p.length - 2].x, p[p.length - 1].y - p[p.length - 2].y));
        return p;
    }


    public void driveAnotherCurve() throws InterruptedException {
        if (chassis == null) return;

        MechChassis.Point[] points = {new MechChassis.Point(120, 35, 90),
                new MechChassis.Point(180, 50, 90),
                new MechChassis.Point(180, 50, 90),
                new MechChassis.Point(200, 75, 90),
                new MechChassis.Point(225, 90, 90),
                new MechChassis.Point(250, 110, 90),
                new MechChassis.Point(200, 135, 90),
                new MechChassis.Point(180, 135, 90),
                new MechChassis.Point(150, 130, 90),
                new MechChassis.Point(130, 110, 90),
                new MechChassis.Point(140, 90, 90),
                new MechChassis.Point(180, 50, 90),
                new MechChassis.Point(240, 35, 90),
                new MechChassis.Point(280, 20, 90),
        };

        chassis.set_init_pos(60, 20, 90);
        chassis.driveThrough(.6, points, false, 10);
    }

    @MenuEntry(label = "driveTo/rotateTo", group = "Test Chassis")
    public void testStraight(EventManager em) throws InterruptedException {
        if (chassis == null) return;
        if (chassis != null && chassis.getGPS() == null) {
            // chassis.set_init_pos(120, 155, 0);
            chassis.set_init_pos(0, 0, 0);
            chassis.configureOdometry(telemetry);
            positionThread = (chassis.getGPS() == null ? null : new Thread(chassis.getGPS()));
            if (positionThread != null) {
                positionThread.start();
            }
        }
        if (interrupted()) return;
        chassis.auto_target_y = chassis.getInit_y_cm();
        chassis.auto_target_x = chassis.getInit_x_cm();
        auto_chassis_heading = chassis.getInit_heading();

        telemetry.addLine().addData("(BACK) Y/A B/X", "+/- Power(%.2f) Degree(%.0f)", auto_chassis_power, auto_rotate_degree).setRetained(true);
        telemetry.addLine().addData("(L-Tr) Y/A B/X", "+/- Y(%.2f) X(%.0f)", chassis.auto_target_y, chassis.auto_target_x).setRetained(true);
        telemetry.addLine().addData("DPAD-UP/DOWN", "+/- distance(%.2f)", auto_chassis_dist).setRetained(true);
        telemetry.addLine().addData("A:Straight", "B:rotate Y:driveTo").setRetained(true);
        chassis.setupTelemetry(telemetry);
        chassis.enableImuTelemetry(cfg);
        chassis.setupIMUTelemetry(telemetry);
        setupTelemetryDiagnostics(telemetry);
        setupTelemetry(telemetry);
        if (shooter != null)
            shooter.setupTelemetry(telemetry);
        // em.updateTelemetry(telemetry, 1000);
        em.updateTelemetry(telemetry, 100);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.START)) {
                    chassis.switchAutoMode();
                } else if (source.isPressed(Button.BACK)) {
                    auto_chassis_power += 0.05;
                    if (auto_chassis_power > 1) auto_chassis_power = 1;
                } else if (source.getTrigger(Events.Side.LEFT) > 0.5) {
                    chassis.auto_target_y += 10;
                } else if (source.getTrigger(Events.Side.RIGHT) > 0.5) { // doPowerShots
                    tZone = TargetZone.ZONE_A;
                    doPowerShots();
                } else if (!source.isPressed(Button.START)) {
                    chassis.driveTo(auto_chassis_power, chassis.auto_target_x, chassis.auto_target_y, auto_rotate_degree, true, 5);
                }
            }
        }, new Button[]{Button.Y});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_chassis_power -= 0.05;
                    if (auto_chassis_power < 0.1) auto_chassis_power = 0.1;
                } else if (source.getTrigger(Events.Side.LEFT) > 0.5) {
                    chassis.auto_target_y -= 10;
                } else if (source.isPressed(Button.LEFT_BUMPER)) {
                    chassis.driveStraight(auto_chassis_power, auto_chassis_dist, auto_rotate_degree, 10);
                } else if (!source.isPressed(Button.START)) {
                    chassis.driveStraightPID(auto_chassis_power, auto_chassis_dist, auto_rotate_degree, 10);
                }
            }
        }, new Button[]{Button.A});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_rotate_degree -= 5;
                } else if (source.getTrigger(Events.Side.LEFT) > 0.5) {
                    chassis.auto_target_x -= 10;
                } else if (!source.isPressed(Button.START)) {
                    chassis.driveStraight(auto_chassis_power, 1000, auto_rotate_degree, 3);
                }
            }
        }, new Button[]{Button.X});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_rotate_degree += 10;
                } else if (source.getTrigger(Events.Side.LEFT) > 0.5) {
                    chassis.auto_target_x += 5;
                } else if (!source.isPressed(Button.START)) {
                    chassis.rotateTo(auto_chassis_power, auto_rotate_degree, 5);
                }
            }
        }, new Button[]{Button.B});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    driveCurve();
                } else {
                    auto_chassis_dist += 10;
                }
            }
        }, new Button[]{Button.DPAD_UP});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    driveCircle();
                } else {
                    auto_chassis_dist -= 10;
                }
            }
        }, new Button[]{Button.DPAD_DOWN});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    driveAnotherCurve();
                } else {

                }
            }
        }, new Button[]{Button.DPAD_RIGHT});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    driveThisPath();
                } else {

                }
            }
        }, new Button[]{Button.DPAD_LEFT});
    }

    @MenuEntry(label = "Auto Rotation", group = "Test Chassis")
    public void testRotationSkyStone(EventManager em) {
        if (chassis == null) return;
        if (chassis != null && chassis.getGPS() == null) {
            chassis.configureOdometry(telemetry);
            positionThread = (chassis.getGPS() == null ? null : new Thread(chassis.getGPS()));
            if (positionThread != null)
                positionThread.start();
        }
        if (interrupted()) return;
        telemetry.addLine().addData("(BACK) Y/A", "+/- Power(%.2f)", auto_chassis_power).setRetained(true);
        telemetry.addLine().addData("(BACK) X/B", "+/- degree(%.2f)", auto_rotate_degree).setRetained(true);
        chassis.setupTelemetry(telemetry);
        setupTelemetryDiagnostics(telemetry);
        // chassis.enableImuTelemetry();
        em.updateTelemetry(telemetry, 100);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_chassis_power += 0.1;
                    if (auto_chassis_power > 1) auto_chassis_power = 1;
                } else {
                    chassis.rotateTo(auto_chassis_power, auto_rotate_degree, 5);
                }
            }
        }, new Button[]{Button.Y});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_chassis_power -= 0.1;
                    if (auto_chassis_power < 0.1) auto_chassis_power = 0.1;
                } else {
                    chassis.rotateTo(auto_chassis_power, auto_rotate_degree, 5);
                }
            }
        }, new Button[]{Button.A});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_rotate_degree -= 10;
                    if (auto_rotate_degree > 150) auto_rotate_degree = 150;
                }
            }
        }, new Button[]{Button.X});
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                if (source.isPressed(Button.BACK)) {
                    auto_rotate_degree += 10;
                    if (auto_rotate_degree < -150) auto_rotate_degree = -150;
                }
            }
        }, new Button[]{Button.B});

    }

    @MenuEntry(label = "Auto-Menu", group = "Competition-Auto")
    public void AutoMenu(EventManager em) {
        telemetry.addLine().addData(" | <X>", "Done").setRetained(true);
        setupTelemetry(telemetry);

        em.updateTelemetry(telemetry, 100);
        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                autoPara.doPowerShots = !autoPara.doPowerShots;
            }
        }, new Button[]{Button.Y});

        em.onButtonDown(new Events.Listener() {
            @Override
            public void buttonDown(EventManager source, Button button) throws InterruptedException {
                autoPara.isDone = true;
            }
        }, new Button[]{Button.X});

    }

    public void initAfterStart() throws InterruptedException {
        if (tZone==TargetZone.UNKNOWN) {// TeleOp
            // shooter.shootOutByRpm(WARM_UP_RPM);
        } else {
            shooter.shootOutByRpm(WARM_UP_RPM_AUTO);
        }
        if (comboGrabber != null && tZone != TargetZone.UNKNOWN) // during autonomous
            comboGrabber.armUpAuto();
        initializeGPSThread();
        if (chassis!=null) {
            batteryVolt = getBatteryVoltage();
            chassis.set_auto_power_scale_by_voltage(batteryVolt);
        }
        if (hopper != null) {
            hopper.blockerUp();
        }
    }

    // Computes the current battery voltage
    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : cfg.getHardwareMap().voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    public void initializeGPSThread() {
        if (chassis == null) return;
        if (positionThread != null) return;
        if (chassis != null && chassis.getGPS() == null) {
            chassis.configureOdometry(telemetry);
        }
        positionThread = (chassis.getGPS() == null ? null : new Thread(chassis.getGPS()));
        if (positionThread != null)
            positionThread.start();
    }

    public void initSetup(ProgramType type, StartPosition startP, Configuration configuration) throws InterruptedException {
        if (hopper != null) {
            hopper.hopperDownCombo();
            TaskManager.processTasks();
        }
        if (chassis == null) return;
        // setup the parameters before the robot configuration
        // 1. enable TFOD or not
        // 2. enable Vuforia or not
        // 3. Robot init position (for auto)
        // 4. initialize TFOD/Vuforia
        // 5. Camera servo position
        side = type;
        startPos = startP;
        switch (type) {
            case TELE_OP:
                useVuforia = false;
                useTfod = false;
                if (cameraDetector != null)
                    cameraDetector.set_cam_pos(cameraDetector.CAM_TELE_OP);
                /*if (chassis.orientationSensor == null) {
                    // chassis.enableImuTelemetry(configuration);

                    // Use the following line if enable IMU after Autonomous
                    chassis.configure_IMUs(configuration, isTeleOpAfterAuto);

                    // Enable the following line only for the debugging purpose
                    // chassis.setupIMUTelemetry(telemetry);
                }*/
                if (chassis != null) {
                    if (isTeleOpAfterAuto) {
                        chassis.initOdoFromJson();
                    } else {
                        // chassis.set_init_pos(side(120), 155, 0);
                        // chassis.set_init_pos(58,23, 0);
                        chassis.set_init_pos(0,0, 0);

                    }
                }
                break;
            case AUTO_RED:
                if (startP == StartPosition.OUT) {
                    if (chassis != null)
                        chassis.set_init_pos(side(300), 23, 0);
                    useVuforia = false;
                    useTfod = true;
                    //setup WebCam servo position for autonomous during initialization
                    if (cameraDetector != null)
                        cameraDetector.set_cam_pos(cameraDetector.CAM_RED_OUT);
                } else { // in position
                    if (chassis != null)
                        chassis.set_init_pos(side(240), 23, 0);
                    useVuforia = false;
                    useTfod = true;
                    //setup WebCam servo position for autonomous during initialization
                    if (cameraDetector != null)
                        cameraDetector.set_cam_pos(cameraDetector.CAM_RED_IN);
                }
                break;
            case AUTO_BLUE:
                if (startP == StartPosition.OUT) {
                    if (chassis != null)
                        chassis.set_init_pos(side(58), 23, 0);
                    useVuforia = false;
                    useTfod = true;
                    //setup WebCam servo position for autonomous during initialization
                    if (cameraDetector != null)
                        cameraDetector.set_cam_pos(cameraDetector.CAM_BLUE_OUT);
                } else { // in position
                    if (chassis != null)
                        chassis.set_init_pos(side(120), 23, 0);
                    useVuforia = false;
                    useTfod = true;
                    //setup WebCam servo position for autonomous during initialization
                    if (cameraDetector != null)
                        cameraDetector.set_cam_pos(cameraDetector.CAM_BLUE_IN);
                }
                break;
            case DIAGNOSIS:
                useVuforia = true;
                break;
        }

        if (chassis != null && chassis.getGPS() == null) {
            chassis.configureOdometry(telemetry);
        }
        configureVisualTool(configuration);

        //setup WebCam servo position for autonomous during initialization
        switch (type) {
            case TELE_OP:
                if (cameraDetector != null)
                    cameraDetector.set_cam_pos(cameraDetector.CAM_TELE_OP);
                break;
            case AUTO_RED:
                if (startP == StartPosition.OUT) {
                    if (cameraDetector != null)
                        cameraDetector.set_cam_pos(cameraDetector.CAM_RED_OUT);
                } else { // in position
                    if (cameraDetector != null)
                        cameraDetector.set_cam_pos(cameraDetector.CAM_RED_IN);
                }
                break;
            case AUTO_BLUE:
                if (startP == StartPosition.OUT) {
                    if (cameraDetector != null)
                        cameraDetector.set_cam_pos(cameraDetector.CAM_BLUE_OUT);
                } else { // in position
                    if (cameraDetector != null)
                        cameraDetector.set_cam_pos(cameraDetector.CAM_BLUE_IN);
                }
                break;
            case DIAGNOSIS:
                break;
        }
    }

    public void hopperDownIntakeCombo() throws InterruptedException {
        if (hopper == null) return;
        final String taskName = "Hopper Down Intake Combo";
        if (!TaskManager.isComplete(taskName)) return;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                hopper.holderIn();
                hopper.HopperTimer.reset();
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return (hopper.HopperTimer.seconds()>0.01);
                    }
                }; }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                hopper.ringLifter.setPower(1);
                hopper.HopperTimer.reset();
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return (hopper.magLow.isPressed() || (hopper.HopperTimer.seconds()>=2.5));
                    }
                }; }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                intake.intakeInBySpeed();
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return (true);
                    }
                }; }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                hopper.ringLifter.setPower(-0.5);
                hopper.HopperTimer.reset();
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return (hopper.HopperTimer.milliseconds()>30);
                    }
                }; }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                if (!hopper.magLow.isPressed()) {
                    hopper.ringLifter.setPower(1);
                }
                hopper.HopperTimer.reset();
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return (hopper.magLow.isPressed() || (hopper.HopperTimer.seconds()>=0.8));
                    }
                }; }}, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                hopper.ringLifter.setPower(0);
                hopper.setTransferIsDown(true);
                return new Progress() {
                    @Override
                    public boolean isDone() {
                        return (true);
                    }
                }; }}, taskName);
    }

    public void setInitPositions(ProgramType s, StartPosition startP) {
        if (chassis == null) return;
        side = s;
        startPos = startP;
        chassis.set_init_pos(side(55), 23, 0);
    }

    public void detectPosition() {//startPos = 1 = out, 2 = in
        // use camera (Tensorflow) to detect position
//        tZone = TargetZone.ZONE_C;
//        return;
        if (cameraDetector == null) {
            tZone = TargetZone.ZONE_A; // assuming zone_A for simulation purpose
            return;
        }
        // tZone = TargetZone.ZONE_A;
        tZone = cameraDetector.getTargetZone();
    }

    public void deliverFirstWobbleGoal() throws InterruptedException {
        // start pos - 1 or 2 (1 inside, 2 outside) <---- probably need to change this to enum?
        // still need to change positions to be far left for blue side
        if (hopper != null) {
            if(tZone == TargetZone.ZONE_C) {
                hopper.hopperUpCombo(true);
            }
            TaskManager.processTasks();
        }
        if (side == ProgramType.AUTO_BLUE) {
            if (tZone == TargetZone.ZONE_A) {//0
                chassis.driveTo(auto_chassis_power, 25, 165, 0, false, 2);
                chassis.rotateToFast(0.8,-50,0.5);
            } else if (tZone == TargetZone.ZONE_B) {//1
                chassis.driveTo(auto_chassis_power, 50, 120, 0, true, 2);
                chassis.driveTo(0.8, 70, 240, 0, true, 2);
            } else if (tZone == TargetZone.ZONE_C) {//4
                chassis.driveTo(1.0, 10, 300, -40, true, 6);
            } else {
                return;
            }

        }
        while (!TaskManager.isComplete("Transfer Up Combo")) {
            TaskManager.processTasks();
        }
        if (comboGrabber != null) {
            comboGrabber.releaseWobbleGoalFastCombo();
            while (!TaskManager.isComplete("release Wobble Goal Fast Combo") && !interrupted()) {
                TaskManager.processTasks();
            }
            comboGrabber.initWobbleGoalCombo();
        }
        //sleep(1000);
    }

    public void deliverFirstWobbleGoalAfterHighGoal() throws InterruptedException {
        // start pos - 1 or 2 (1 inside, 2 outside) <---- probably need to change this to enum?
        // still need to change positions to be far left for blue side
        if (comboGrabber!=null)
            comboGrabber.armUpLow();
        if (side == ProgramType.AUTO_BLUE) {
            if (tZone == TargetZone.ZONE_A) {//0
                chassis.driveTo(0.8, 25, 175, -20, false, 3);
                chassis.rawRotateTo(0.4,-30,false,1);
            } else if (tZone == TargetZone.ZONE_B) {//1
                chassis.driveTo(0.8, 65, 230, 0, true, 3);
            } else if (tZone == TargetZone.ZONE_C) {//4
                chassis.driveTo(1.0, 5, 290, 0, false, 3); // no rotation to make it faster
            } else {
                return;
            }

        }

        if (comboGrabber != null) {
            if(tZone != TargetZone.ZONE_C) // Zone A and B
            {
                comboGrabber.releaseWobbleGoalCombo();
                while (!TaskManager.isComplete("release Wobble Goal Combo") && !interrupted()) {
                    TaskManager.processTasks();
                }
                if (tZone == TargetZone.ZONE_A) {
                    chassis.driveStraight(0.5, -20, 0, 1);
                    //chassis.driveTo(auto_chassis_power, chassis.odo_x_pos_cm(), chassis.odo_y_pos_cm() - 20, chassis.odo_heading(), false, 1);
                    chassis.rawRotateTo(0.4,-5,false,1);
                }
            }
            else
            {
                comboGrabber.releaseWobbleGoalFastCombo();
                while (!TaskManager.isComplete("release Wobble Goal Fast Combo") && !interrupted()) {
                    TaskManager.processTasks();
                }
                //comboGrabber.initWobbleGoalCombo();
            }
            comboGrabber.initWobbleGoalCombo();
            TaskManager.processTasks();
        }
        /*if (tZone == TargetZone.ZONE_C) {
            chassis.rawRotateTo(0.3, -2.5, true, 0.5);
        }*/

        //sleep(1000);
    }

    public void autoShoot() throws InterruptedException {
        if (shooter == null || hopper == null) return;
        double iniTime = System.currentTimeMillis();
        int target = shooter.getShooterSpeed();
        // Stage-1 Make sure that rpm exceeds the target
        if (shooter.getCurrentRPM() < target - 200) {
            while (target - shooter.getCurrentRPM() > 0 && (System.currentTimeMillis() - iniTime < 3000)) {
                sleep(10);
            }
            sleep(200);
            while (target - shooter.getCurrentRPM() < 0 && (System.currentTimeMillis() - iniTime < 1500)) {
                sleep(10);
            }
            sleep(200);
            // Stage-2 make sure rpm difference is within 11 error range
            while (Math.abs(shooter.getCurrentRPM() - target) > 11 && (System.currentTimeMillis() - iniTime < 1000)) { // timeout 0.5 sec
                sleep(10);
            }
            sleep(100);
        } else {
            shooter.shootOutByRpm(target - 100);
        }

        // Stage-3 make sure rpm difference is within 11 error range
        while (Math.abs(shooter.getCurrentRPM() - target) > 11 && (System.currentTimeMillis() - iniTime < 500)) { // timeout 5 sec
            sleep(5);
        }
        if (useIMUforOdometryAngleCorrection){
            chassis.getGPS().correctAngleUsingIMU();
        }
        //shooter.shootOutByRpm(target - 70);
        hopper.feederAuto();
    }

    public boolean autoShootFast(boolean forAuto) throws InterruptedException {
        if (shooter == null || hopper == null) return false;
        double iniTime = System.currentTimeMillis();
        int target = shooter.getShooterSpeed();
        if (forAuto) {
            shooter.shootOutByRpm(target - 60);
        } else {
            // shooter.shootOutByRpm(SEMI_AUTO_RPM-150);
        }
        int timeout_ms = (forAuto?500:500);
        // Stage-2 make sure rpm difference is within 11 error range
        while (Math.abs(shooter.getCurrentRPM() - target) > 11 && (System.currentTimeMillis() - iniTime < timeout_ms)) { // timeout 5 sec
            sleep(5);
        }
        // for TeleOp, timeout will not shoot
        //if (Math.abs(shooter.getCurrentRPM() - target) > 20 && forAuto==false)
        //    return false;

        if (useIMUforOdometryAngleCorrection){
            chassis.getGPS().correctAngleUsingIMU();
        }
        if (forAuto) {
            shooter.shootOutByRpm(target - 60);
        } else {
            // shooter.shootOutByRpm(SEMI_AUTO_RPM-150);
        }
        hopper.feederAuto();
        return true;
    }

    public void doPowerShots() throws InterruptedException {
        if (tZone == TargetZone.ZONE_A) {
            chassis.driveTo(auto_chassis_power, side(60), 170, chassis.odo_heading(), false, 1);
            chassis.rotateTo(1, 0,1);
        } else if (tZone == TargetZone.ZONE_C) {
            chassis.driveStraight(1.0, -100, chassis.odo_heading() + 10, 5);
        }

        shooter.shootOutByRpm(WARM_UP_RPM_POWER_SHOT);
        if (tZone == TargetZone.ZONE_C) {
            chassis.driveTo(1.0, side(130), 170, 0, false, 3);
        } else if (tZone == TargetZone.ZONE_A) {
            chassis.driveTo(.8, side(122), 170, 0, false, 5);
        } else { // Zone-B
            chassis.driveTo(.8, side(124), 175, 0, false, 5);
        }

        if (hopper != null) {
            hopper.hopperUpCombo(true);
            TaskManager.processTasks();
        }
        while (!TaskManager.isComplete("Transfer Up Combo")) {
            TaskManager.processTasks();
        }
//        if (tZone == TargetZone.ZONE_C) { // temporarily disable the shooting
//            shooter.shootOutByRpm(0);
//            return;
//        }
            doPowerShotsSemi(3, true);
       // rotateToTargetAndStartShooter(MechChassis.ShootingTarget.PSHOT_L, false);
        //shoot
        //autoShoot();
        //sleep(500);
        //rotateToTargetAndStartShooter(MechChassis.ShootingTarget.PSHOT_M, false);
        //chassis.driveTo(.55, side(150), 170, 0, false,  2);
        //shoot
        //autoShoot();
        //sleep(500);
        //rotateToTargetAndStartShooter(MechChassis.ShootingTarget.PSHOT_R, false);
        //chassis.driveTo(.55, side(170), 170, 0, false,  2);
        //shoot
        //autoShoot();
        //sleep(200);
        shooter.shootOutByRpm(0);
    }

    public void doHighGoalsAndPowerShots(int numHighGoals, int numPowerShots, boolean keepPos) throws InterruptedException {
        shooter.shootOutByRpm(WARM_UP_RPM_AUTO);


        if (hopper != null) {
            if (hopper.getTransferIsDown())
                hopper.hopperUpCombo(true);
            TaskManager.processTasks();
        }
        if (intake != null)
            intake.stop();
        if (side!=ProgramType.TELE_OP && runtimeAuto.seconds() > 29){ return;}
        if (!keepPos) {
            if (tZone == TargetZone.ZONE_B || tZone == TargetZone.ZONE_C && numHighGoals == 3) {
                chassis.driveTo(.6, side(45), 165, 0, true, 4);
            } else if (tZone != TargetZone.UNKNOWN) {
                if (side!=ProgramType.TELE_OP &&runtimeAuto.seconds() > 29) {
                    return;
                }
                chassis.driveTo(.6, side(90), 165, 0, true, 4);
            }
            }
        while (!TaskManager.isComplete("Transfer Up Combo")) {
            TaskManager.processTasks();
        }

        // need to do something about this
        rotateToTargetAndStartShooter(MechChassis.ShootingTarget.TOWER, false);
        //shoot
        for (int i = 0; i < numHighGoals; i++) {
            if (side!=ProgramType.TELE_OP && runtimeAuto.seconds() > 29){ return;}
            if (i == 0) {
                autoShootFast(true);
            } else {
                autoShootFast(true);
            }
            if (i < numHighGoals - 1)
                sleep(200);
        }
        if (numPowerShots > 0) {
            if (side!=ProgramType.TELE_OP && runtimeAuto.seconds() > 29){ return;}
            shooter.shootOutByRpm(WARM_UP_RPM_POWER_SHOT);
            rotateToTargetAndStartShooter(MechChassis.ShootingTarget.PSHOT_L, false);
            //shoot
            autoShootFast(true);
        }
        if (numPowerShots > 1) {
            if (side!=ProgramType.TELE_OP && runtimeAuto.seconds() > 29){ return;}
            rotateToTargetAndStartShooter(MechChassis.ShootingTarget.PSHOT_M, false);
            //chassis.driveTo(.55, side(150), 170, 0, false,  2);
            //shoot
            autoShootFast(true);
            //sleep(500);
        }
        if (numPowerShots > 2) {
            if (side!=ProgramType.TELE_OP && runtimeAuto.seconds() > 29){ return;}
            rotateToTargetAndStartShooter(MechChassis.ShootingTarget.PSHOT_R, false);
            //chassis.driveTo(.55, side(170), 170, 0, false,  2);
            //shoot
            autoShootFast(true);
            sleep(200);
            shooter.shootOutByRpm(0);
        }

        if (tZone == TargetZone.UNKNOWN)
            shooter.shootOutByRpm(WARM_UP_RPM_AUTO); // for teleop keep the shooter at WARM_UP_RPM
        else
            shooter.shootOutByRpm(0);
    }

    public void doHighGoalsSemi(boolean angleCorrection, int nshots) throws InterruptedException {
        shooter.shootOutByRpm(SEMI_AUTO_RPM);
        shooting_rpm = SEMI_AUTO_RPM;
        boolean hopperMoving = hopper.getTransferIsDown();
        if (hopper != null) {
            if (hopper.getTransferIsDown())
                hopper.hopperUpCombo(true);
            TaskManager.processTasks();
        }
        if (intake!=null)
            intake.stop();
        while (!TaskManager.isComplete("Transfer Up Combo")) {
            TaskManager.processTasks();
        }
        if (hopperMoving) sleep(300); // delay 0.3 sec for hopper to lock up

        if(angleCorrection){
            double heading = SEMI_HIGHGOAL_HEADING;
            if (Math.abs(chassis.odo_heading() - heading) > 0.8) {
                if (Math.abs(chassis.odo_heading() - heading) > 10) {
                    chassis.rotateTo(0.3, heading);
                    sleep(100);
                }
                int i=0;
                while (Math.abs(chassis.odo_heading() - heading)>1 && i<2) {
                    chassis.rawRotateTo(chassis.chassisAligmentPowerMin, heading, false, 0.5);
                    i++;
                }
                //sleep(200);
            }
        }

        if (hopper != null) {
            hopper.blockerDown();
        }
        // chassis.resetOdometry(true); // use rangeSensor to correct Odometry
        //shoot
        for (int i=0; i<nshots; i++) {
            if (i==0) {
                autoShoot();
                shooter.shootOutByRpm(SEMI_AUTO_RPM-40);
                sleep(200);
            }
            else {
                autoShootFast(false);
                shooter.shootOutByRpm(SEMI_AUTO_RPM-40);
                if (i==1) {
                    sleep(250);
                }
            }
        }
        shooter.shootOutByRpm(WARM_UP_RPM);
    }

    public void doPowerShotsSemi(int n, boolean angleCollection) throws InterruptedException {
        shooter.shootOutByRpm(SEMI_POWER_SHOT_RPM);
        shooting_rpm = SEMI_POWER_SHOT_RPM;
        if (hopper != null && hopper.getTransferIsDown()) {
            hopper.hopperUpCombo(true);
            TaskManager.processTasks();
        }
        if (intake!=null)
            intake.stop();
        while (!TaskManager.isComplete("Transfer Up Combo")) {
            TaskManager.processTasks();
        }
        double target_heading = chassis.odo_heading();

        if(angleCollection){
            target_heading=2.3;
            if (Math.abs(chassis.odo_heading() - target_heading) >= 0.8) {
                if (Math.abs(chassis.odo_heading() - target_heading) > 10) {
                    chassis.rotateTo(0.3, target_heading);
                    sleep(100);
                }
                int i=0;
                while (Math.abs(chassis.odo_heading() - target_heading)>0.8 && i<3) {
                    chassis.rawRotateTo(0.18, target_heading, false, 0.5);
                    i++;
                }
                sleep(100);
            } else sleep(300);
        }


        //chassis.resetOdometry(true); // use rangeSensor to correct Odometry
        //shoot

        hopper.feederAuto();
        if (n==1) {
            shooter.shootOutByRpm(SEMI_POWER_SHOT_RPM);
            return;
        }
        shooter.shootOutByRpm(SEMI_POWER_SHOT_RPM-40);
        //sleep(200);
        // move to center power shot
        //shooter.shootOutByRpm(SEMI_POWER_SHOT_RPM-60);
        //chassis.rawRotateTo(0.25, chassis.odo_heading()+3.5, false, 1);
        // chassis.driveStraight(0.5, 19, 90, 2);
        chassis.driveTo(0.5, chassis.odo_x_pos_cm()+19,chassis.odo_y_pos_cm(),target_heading,false,1);
        if(angleCollection){
            if (Math.abs(chassis.odo_heading() - target_heading) >= 0.5) {
                if (Math.abs(chassis.odo_heading() - target_heading) > 10) {
                    chassis.rotateTo(0.3, target_heading);
                    sleep(100);
                }
                int i=0;
                while (Math.abs(chassis.odo_heading() - target_heading)>0.8 && i<2) {
                    chassis.rawRotateTo(chassis.chassisAligmentPowerMin, target_heading, false, 0.5);
                    i++;
                }
                sleep(100);
            }
        } else sleep(200);
        hopper.feederAuto();
        if (n==2) {
            shooter.shootOutByRpm(SEMI_POWER_SHOT_RPM);
            return;
        }
        shooter.shootOutByRpm(SEMI_POWER_SHOT_RPM-40);
        //sleep(500);
        // move to right power shot
        //shooter.shootOutByRpm(SEMI_POWER_SHOT_RPM-60);
        //chassis.rawRotateTo(0.25, chassis.odo_heading()+3.5, false, 1);

        //chassis.driveStraight(0.5, 19, 90, 2);
        //chassis.driveTo(0.5, chassis.odo_x_pos_cm()+19,chassis.odo_y_pos_cm(),target_heading-0.5,false,1);
        chassis.driveTo(0.5, chassis.odo_x_pos_cm()+19,chassis.odo_y_pos_cm(),chassis.odo_heading(),false,1);

        if(angleCollection){
            if (Math.abs(chassis.odo_heading() - target_heading) >= 0.5) {
                if (Math.abs(chassis.odo_heading() - target_heading) > 10) {
                    chassis.rotateTo(0.3, target_heading);
                    sleep(100);
                }
                int i=0;
                while (Math.abs(chassis.odo_heading() - target_heading)>0.8 && i<2) {
                    chassis.rawRotateTo(chassis.chassisAligmentPowerMin, target_heading, false, 0.5);
                    i++;
                }
                sleep(100);
            }
        } else sleep(200);
        hopper.feederAuto();
        sleep(100);
        shooter.shootOutByRpm(SEMI_POWER_SHOT_RPM);
    }
    public void doPowerShotsSemiNew(int n, boolean angleCollection, boolean driverAligned) throws InterruptedException {//auto power shots
        shooter.shootOutByRpm(SEMI_POWER_SHOT_RPM);
        shooting_rpm = SEMI_POWER_SHOT_RPM;
        if (hopper != null && hopper.getTransferIsDown()) {
            hopper.hopperUpCombo(true);
            TaskManager.processTasks();
        }
        if (intake!=null)
            intake.stop();
        double target_heading=(angleCollection?2.3:chassis.odo_heading());
        if (driverAligned) {
            // assume robot is facing 0 degree by driver. Ues this to correct IMU
            double imu_heading = chassis.orientationSensor.getHeading();
            if (chassis.getGPS()!=null) {
                chassis.getGPS().correctHeading(-imu_heading);
                sleep(100);
            }
            target_heading=chassis.odo_heading()+2.6;
        } else {
            chassis.rotateTo(0.3, 0);
            sleep(200);
            target_heading=(angleCollection?2.3:chassis.odo_heading());
        }
        double idealRightDist = 62; // 45 cm at Hans field; 62 cm at Winston's house
        double crab_power=0.5;
        double timeout=3.0;
        for (int i=0; i<5; i++) {
            double rightDistF = chassis.getDistance(SwerveChassis.Direction.RIGHT_FRONT);
            double rightDistB = chassis.getDistance(SwerveChassis.Direction.RIGHT_BACK);
            if (rightDistF>100){
                rightDistF = rightDistB;
            }
            if (rightDistB>100){
                rightDistB = rightDistF;
                if (rightDistF>100){
                    return;
                }
            }
            double rightDist = (rightDistF + rightDistB) / 2;
            if (Math.abs(rightDist-idealRightDist)<2) break;
            chassis.driveTo(crab_power, chassis.odo_x_pos_cm() - (idealRightDist - rightDist), chassis.odo_y_pos_cm(),
                    chassis.odo_heading(), false, false, timeout);

            crab_power-=0.05;
            timeout=1.0;
            sleep(100);
        }
        // chassis.rotateTo(0.3, 0);//delete?
        double angle_error = 0.5;
        double LOCAL_ALIGNMENT_POWER = 0.175;
        double ALIGN_ITER = 5;
        if(angleCollection){
            if (Math.abs(chassis.odo_heading() - target_heading) >= angle_error) {
                if (Math.abs(chassis.odo_heading() - target_heading) > 10) {
                    chassis.rotateTo(0.3, target_heading);
                    sleep(100);
                }
                int i=0;
                while (Math.abs(chassis.odo_heading() - target_heading)>angle_error && i<ALIGN_ITER) {
                    chassis.rawRotateTo(LOCAL_ALIGNMENT_POWER, target_heading, false, 0.5);
                    sleep(50);
                    i++;
                }
                sleep(100);
            } else sleep(300);
        }

        while (!TaskManager.isComplete("Transfer Up Combo")) {
            TaskManager.processTasks();
        }

        //chassis.resetOdometry(true); // use rangeSensor to correct Odometry
        //shoot

        hopper.feederAuto();
        if (n==1) {
            shooter.shootOutByRpm(SEMI_POWER_SHOT_RPM);
            return;
        }
        shooter.shootOutByRpm(SEMI_POWER_SHOT_RPM-40);
        //sleep(200);
        // move to center power shot
        //shooter.shootOutByRpm(SEMI_POWER_SHOT_RPM-60);
        //chassis.rawRotateTo(0.25, chassis.odo_heading()+3.5, false, 1);
        // chassis.driveStraight(0.5, 19, 90, 2);
        chassis.driveTo(0.5, chassis.odo_x_pos_cm() - 17, chassis.odo_y_pos_cm(), chassis.odo_heading(),
                false, false,2);

        if(angleCollection){
            sleep(100);
            if (Math.abs(chassis.odo_heading() - target_heading) >= angle_error) {
                if (Math.abs(chassis.odo_heading() - target_heading) > 10) {
                    chassis.rotateTo(0.3, target_heading);
                    sleep(100);
                }
                int i=0;
                while (Math.abs(chassis.odo_heading() - target_heading)>angle_error && i<ALIGN_ITER) {
                    chassis.rawRotateTo(LOCAL_ALIGNMENT_POWER, target_heading, false, 0.5);
                    sleep(50);
                    i++;
                }
                sleep(100);
            } else sleep(300);
        }
        hopper.feederAuto();
        if (n==2) {
            shooter.shootOutByRpm(SEMI_POWER_SHOT_RPM);
            return;
        }
        shooter.shootOutByRpm(SEMI_POWER_SHOT_RPM-40);
        //sleep(500);
        // move to right power shot
        //shooter.shootOutByRpm(SEMI_POWER_SHOT_RPM-60);
        //chassis.rawRotateTo(0.25, chassis.odo_heading()+3.5, false, 1);

        //chassis.driveStraight(0.5, 19, 90, 2);
        chassis.driveTo(0.5, chassis.odo_x_pos_cm() - 19, chassis.odo_y_pos_cm(), chassis.odo_heading(),
                false, false,2);

        if(angleCollection){
            sleep(100);
            if (Math.abs(chassis.odo_heading() - target_heading) >= angle_error) {
                if (Math.abs(chassis.odo_heading() - target_heading) > 10) {
                    chassis.rotateTo(0.3, target_heading);
                    sleep(100);
                }
                int i=0;
                while (Math.abs(chassis.odo_heading() - target_heading)>angle_error && i<ALIGN_ITER) {
                    chassis.rawRotateTo(LOCAL_ALIGNMENT_POWER, target_heading, false, 0.5);
                    sleep(50);
                    i++;
                }
                sleep(100);
            } else sleep(300);
        }
        hopper.feederAuto();
        shooter.shootOutByRpm(SEMI_POWER_SHOT_RPM);
    }

    public void getSecondWobbleGoal() throws InterruptedException {
        if (hopper != null) {
            hopper.hopperDownCombo();
            TaskManager.processTasks();
        }
        if (tZone == TargetZone.ZONE_C) {
            chassis.driveTo(1.0, side(165), 33, 0, true,  5);
        }
        else {
            chassis.driveTo(auto_chassis_power, side(165), 32, 0, true, 5);
        }
        if(startPos == StartPosition.OUT){
            if (tZone == TargetZone.ZONE_C) {
                chassis.driveTo(0.6, side(105), 29, 0, false, 3);
            }
            else {
                chassis.driveTo(0.6, side(105), 27, 0, false, 3);
            }
        } else {
            chassis.driveTo(auto_chassis_power, side(47), 29, 0, true,  3);
        }
        while (!TaskManager.isComplete("Transfer Down Combo")) {
            TaskManager.processTasks();
        }
        //grab the wobble goal
        if (!simulation_mode) {
            autoGrabBottomWobbleGoal();
        }

        // sleep(1000);
    }
    public void getSecondWobbleGoalAfterHighGoal() throws InterruptedException {
        if (hopper != null) {
            hopper.hopperDownCombo();
            TaskManager.processTasks();
        }

        if(tZone == TargetZone.ZONE_B){
            chassis.driveTo(auto_chassis_power, side(40), 170, 0, false,  5);
        }
        if(tZone == TargetZone.ZONE_B || tZone == TargetZone.ZONE_C){
            shooter.shootOutByRpm(WARM_UP_RPM_AUTO);
        }
        if(tZone == TargetZone.ZONE_C)
        {
            chassis.driveTo(1, side(60), 37, 0, false, 4);
        }
        else
        {
            chassis.driveTo(auto_chassis_power, side(70), 37, 0, true, 5);
            chassis.rotateTo(auto_chassis_align_power, 0);
        }
        if(startPos == StartPosition.OUT){
            if (tZone == TargetZone.ZONE_C){
                chassis.driveTo(0.6, side(102), 29, 0, false, 2);
            } else if (tZone == TargetZone.ZONE_B){
                chassis.driveTo(0.6, side(102), 29, 0, false, 2);
            } else { // ZONE_A
                chassis.driveTo(0.6, side(100), 27, 0, false, 2);
            }
        } else {
            chassis.driveTo(auto_chassis_power, side(47), 30, 0, true,  3);
        }
        while (!TaskManager.isComplete("Transfer Down Combo")) {
            TaskManager.processTasks();
        }
        //grab the wobble goal
        if (!simulation_mode) {
            autoGrabBottomWobbleGoal();
        }

        // sleep(1000);
    }
    public void deliverSecondWobbleGoalAndShootBonusRings() throws InterruptedException { // we may need to go around the other wobble goal

        // need to change positions
        if (side == ProgramType.AUTO_BLUE) {
            if (tZone == TargetZone.ZONE_A) {//0
                // chassis.driveTo(.8, side(30), 40, 0, false, 2);
                if(comboGrabber != null)
                        comboGrabber.armUpLow();
                chassis.driveTo(0.9, side(12), 165, -20, false, 3);
            } else if (tZone == TargetZone.ZONE_B) {//1
                shooter.shootOutByRpm(WARM_UP_RPM_AUTO);
                intake.intakeInBySpeed();
                chassis.driveTo(0.8, side(75), 165, 0, false, 5);
                sleep(500); //to allow time for intaking the bonus ring
                intake.stop();
                autoShootHighGoal(1, true);
                if (comboGrabber!=null)
                    comboGrabber.armUpLow();
                chassis.driveTo(0.8, side(75), 230, 0, false, 5);
                while (!TaskManager.isComplete("Transfer Down Combo") && !interrupted()) {
                    TaskManager.processTasks();
                }
            } else if (tZone == TargetZone.ZONE_C) {//4
                shooter.shootOutByRpm(WARM_UP_RPM_AUTO);
                if (hopper != null) {
                    hopper.ringBarDown();
                }
                //chassis.driveTo(.8, side(30), 60, 0, false, 5);
                chassis.driveTo(0.6, side(87), 90, 0, false, 2);
                autoIntakeRingsNew(3, true);
                if (hopper != null) {
                    hopper.ringBarUp();
                    hopper.hopperUpCombo(true);
                    TaskManager.processTasks();
                }
                chassis.driveTo(0.5, side(87), 165, 0, false, 2);
                autoShootHighGoal(3, true);
                if (comboGrabber!=null)
                    comboGrabber.armUpLow();
                shooter.shootOutByRpm(WARM_UP_RPM_AUTO);
                while (!TaskManager.isComplete("Transfer Down Combo")) {
                    TaskManager.processTasks();
                }
                intake.intakeInBySpeed();
                chassis.driveTo(1.0, side(5), 287, 0, false, 2.5);
                intake.stop();
            } else {
                return;
            }
        }
        if (comboGrabber!=null) {
            if(tZone != TargetZone.ZONE_C) // Zone A and B
            {
                comboGrabber.releaseWobbleGoalCombo();
                while (!TaskManager.isComplete("release Wobble Goal Combo") && !interrupted()) {
                    TaskManager.processTasks();
                }
            }
            else
            {
                comboGrabber.releaseWobbleGoalFastCombo();
                while (!TaskManager.isComplete("release Wobble Goal Fast Combo") && !interrupted()) {
                    TaskManager.processTasks();
                }
                //comboGrabber.initWobbleGoalCombo();
            }
        }
        //sleep(1000);
    }
    public void autoShootHighGoal(int n, boolean keepPos) throws InterruptedException {
        shooter.shootOutByRpm(WARM_UP_RPM_AUTO);
        hopper.hopperUpCombo(true);
        TaskManager.processTasks();
        doHighGoalsAndPowerShots(n, 0, keepPos);
        hopper.hopperDownCombo();
        TaskManager.processTasks();
    }
    public void park() throws InterruptedException {
        if (tZone==TargetZone.ZONE_A){
            chassis.driveTo(1.0, chassis.odo_x_pos_cm(), chassis.odo_y_pos_cm() - 20, 0, false, 1);
            chassis.driveTo(1, side(110), 165, 0, false, 2);
        }
        if (tZone==TargetZone.ZONE_C){
            shooter.shootOutByRpm(WARM_UP_RPM_AUTO);
            hopper.hopperUpCombo(true);
            TaskManager.processTasks();
            if (runtimeAuto.seconds() < 26) {
                chassis.driveTo(1.0, side(80), 185, 0, false, 5);
                comboGrabber.initWobbleGoalCombo();
                TaskManager.processTasks();
                autoShootHighGoal(2, true);
                comboGrabber.initWobbleGoalCombo();
                TaskManager.processTasks();
                chassis.driveTo(1.0, chassis.odo_x_pos_cm()+50, chassis.odo_y_pos_cm() + 30, chassis.getCurHeading(), false, 2);
            }
            else {
                chassis.driveTo(1.0, chassis.odo_x_pos_cm()+20, 220, chassis.getCurHeading(), false, 2);
            }
        }
        else if (tZone==TargetZone.ZONE_B)
        {
            chassis.driveTo(1.0, Math.max(120, Math.min(chassis.odo_x_pos_cm(), 170)), 220, chassis.getCurHeading(), false,  0.9);
        }
        else
        {
            chassis.driveTo(1.0, Math.max(120, Math.min(chassis.odo_x_pos_cm(), 170)), 190, chassis.getCurHeading(), false,  0.9);
        }
        while (!TaskManager.isComplete("Transfer Down Combo")) {
            TaskManager.processTasks();
        }
    }
    public double side( double x){
        if (side == ProgramType.AUTO_RED){
            return 360 - x;
        }
        return x;
    }
    public void autoGrabBottomWobbleGoal() throws InterruptedException {
        if (simulation_mode || chassis==null) return;
        comboGrabber.grabberOpen();
        comboGrabber.armUp();
        sleep(200);
        comboGrabber.armDown();
        sleep(200);
        chassis.yMove(1, 0.5);
        sleep(100);
        chassis.yMove(1, 0.2);
        if (comboGrabber!=null) {
            comboGrabber.grabWobbleGoalCombo(false);
            while (!TaskManager.isComplete("grab Wobble Goal Combo") && !interrupted()) {
                TaskManager.processTasks();
            }
        }
        //chassis.stop();
    }
    public void autoGrabHighWobbleGoal(boolean autoTurn) throws InterruptedException {
        if (simulation_mode || chassis==null) return;
        /* comboGrabber.grabberOpen();
        sleep(100);
        comboGrabber.armDown();
        sleep(200);
         */
        chassis.yMove(1, 0.3);
        sleep(200);
        chassis.stop();
        if (comboGrabber!=null) {
            comboGrabber.grabWobbleGoalFastCombo();
            while (!TaskManager.isComplete("grab Wobble Goal Fast Combo") && !interrupted()) {
                TaskManager.processTasks();
            }
        }
        comboGrabber.raiseWobbleGoalCombo();
        TaskManager.processTasks();
        if (autoTurn) {
            sleep(200);
            chassis.rawRotateTo(0.8, 130, false, 2);
            chassis.rawRotateTo(0.2, 165, false, 2);
        }
    }

    public void autoReleaseHighWobbleGoal() throws InterruptedException {
        if (simulation_mode || chassis==null) return;
        if (comboGrabber!=null) {
            comboGrabber.grabberOpen();
        }
        sleep(200);
        comboGrabber.releaseWobbleGoalCombo();
        TaskManager.processTasks();
        chassis.rawRotateTo(0.8,50,false, 2);
        chassis.rawRotateTo(0.2,15,false, 2);
    }

    public void autoIntakeRingsNew(int n, boolean fromAuto) throws InterruptedException {
        if (simulation_mode || chassis==null) return;

        if (hopper != null) {
            hopper.ringBarDown();
            sleep(200);
        }
        if (!fromAuto) {
            chassis.driveTo(0.6, side(87), 90, 0, false, 2);
        }
        intake.intakeIn();
        chassis.driveTo(0.25, chassis.odo_x_pos_cm(), 89+n*12, 2);
        if(hopper != null) {
            hopper.ringBarUp();
            sleep(200);
        }
        chassis.stop();
        sleep(800);
        //sleep(1000);
        //hopper.transferShakeCombo();
        intake.stop();
    }

    public void autoIntakeRings(int n, boolean callFromAuto) throws InterruptedException {
        if (simulation_mode || chassis==null) return;
        if(!callFromAuto) {
            if (hopper != null) {
                hopper.ringBarDown();
                sleep(200);
            }
            chassis.yMove(1, 1.0);
            sleep(400);
            chassis.stop();
            chassis.yMove(1, -0.4);
            sleep(300);
        }
        chassis.yMove(1, -0.35);
        sleep(250);
        chassis.yMove(1, 0.17);
        intake.intakeIn();
        for (int i = 0; i < n; i++) {
            sleep(650);
            if(i+2==n)
                chassis.stop();
        }
        if(hopper != null) {
            hopper.ringBarUp();
            sleep(200);
        }
        chassis.stop();
        sleep(400);
        //sleep(1000);
        //hopper.transferShakeCombo();
        intake.stop();
        /*if (n>1) {
            // backup a little bit to prevent getting fourth ring
            //chassis.yMove(-1, 0.30);
            //sleep(100);
            //chassis.stop();
            intake.intakeOut();
            sleep(400);
            intake.stop();
        }*/
    }

    public void endGameGrabCombo() throws InterruptedException {
        if (comboGrabber==null) return;
        comboGrabber.releaseWobbleGoalCombo();
        while (!TaskManager.isComplete("release Wobble Goal Combo") && !interrupted()) {
            TaskManager.processTasks();
        }
        chassis.yMove(-1, 0.2);
        sleep(400);
        chassis.yMove(1, 0.2);
        sleep(450);
        chassis.stop();
        comboGrabber.grabWobbleGoalCombo(true);
        while (!TaskManager.isComplete("grab Wobble Goal Combo") && !interrupted()) {
            TaskManager.processTasks();
        }
    }

    public void rotateToTargetAndStartShooter(MechChassis.ShootingTarget target, boolean useVuforia) {
        if (useIMUforOdometryAngleCorrection){
            chassis.getGPS().correctAngleUsingIMU();
        }
        double target_x = 0;
        double target_y = 360;
        double target_height = 0;
        double[] vuforia_position = {-1,-1,-1};
        if(useVuforia&&cameraDetector!=null)
        {
            vuforia_position = cameraDetector.getPositionFromVuforia();
        }
        if(vuforia_position[0] == -1 && vuforia_position[1] == -1)
        {
            useVuforia = false;
        }
        double[] shooter_position = new double[]{
                (useVuforia?vuforia_position[0] + webcam_offset_x + shooter_offset : chassis.odo_x_pos_cm() + shooter_offset),
                (useVuforia?vuforia_position[1] + webcam_offset_y : chassis.odo_y_pos_cm())
        };
        switch (target) {
            case TOWER:
                target_x = 90; // 90;
                target_height = 92;
                break;
            case PSHOT_L:
                target_x = 133;
                target_height = 79;
                break;
            case PSHOT_M:
                target_x = 152;
                target_height = 78.5;
                break;
            case PSHOT_R:
                target_x = 171;
                target_height = 78.5;
                break;
        }
        // start the shooter with expected RPM
        double dx = target_x - shooter_position[0];
        double dy = target_y - shooter_position[1];
        shooting_dist = Math.hypot(dx,dy);
        double v = getVelocityToShoot(shooting_dist, target_height);
        double rpm = getRpmFromVelocity(v)-40;
        shooting_rpm = rpm;
        shooter.shootOutByRpm(rpm);
        // Use current position (odo_x_pos_cm(), odo_y_pos_cm()) and (target_x, target_y) to determine the rotateTo() angle
        shooting_angle = Math.toDegrees(Math.atan2(target_x - chassis.odo_x_pos_cm() - shooter_offset, target_y - chassis.odo_y_pos_cm()));
        // to-do: need to adjust the angle (to right) when dist is > 200 cm 0.047 degree/cm
        //if (shooting_dist>200) {
        //    shooting_angle += (shooting_dist-200)*0.047;
        //}
        double rpm_shift = getShootingAngleErrorFromRPM(rpm);
        shooting_angle += shooterAngleOffset;
        try {
            if (Math.abs(chassis.odo_heading() - shooting_angle) < 15) {
                chassis.rawRotateTo(chassis.chassisAligmentPowerMin, shooting_angle, false, 2);
            } else if (Math.abs(chassis.odo_heading() - shooting_angle) > 0.8) {
                chassis.rotateTo(0.35, shooting_angle);
                sleep(200);
                int i=0;
                while (Math.abs(chassis.odo_heading() - shooting_angle)>0.6 && i<3) {
                    chassis.rawRotateTo(chassis.chassisAligmentPowerMin, shooting_angle, false, 1);
                    i++;
                }
                //sleep(200);
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        // Use current position (odo_x_pos_cm(), odo_y_pos_cm()) and (target_x, target_y) to determine the rotateTo() angle
    }

    public void initializePositionFromVuforia()
    {
        if(cameraDetector == null)
        {
            return;
        }
        double[] vuforia_position;
        vuforia_position = cameraDetector.getPositionFromVuforia();
        if(vuforia_position[0] == -1 && vuforia_position[1] == -1)
        {
            chassis.set_init_pos(vuforia_position[0]+webcam_offset_x,
                    vuforia_position[1]+webcam_offset_y,
                    vuforia_position[2]);
        }
    }

    public double getRpmFromVelocity(double velocity){
        double a = 159.647; // 225.686;
        double b = 290.359; // -35.9631;
        double ideal_rpm = a*velocity+b-10;
        double real_rpm = (int)(ideal_rpm/20) * 20;
        double error = ideal_rpm - real_rpm;
        if (error > 10){
            real_rpm += 20;
        }
        if (real_rpm>MAX_RPM) // cap rpm to MAX_RPM to fit 16 feet limit
            real_rpm = MAX_RPM;
        return real_rpm;
    }

    public  double getVelocityToShoot(double dHorizontal, double dVertical){
        // (dx, dy) is the location delta from the target to the the robot
        // height is the target height to hit

        //unit conversion - meters, inches, centimeters
        dHorizontal = dHorizontal/ 100.;
        dVertical  = dVertical /100.- 0.381;
        double shooterAngle = 22;
        double vSquared = (4.905/Math.cos(Math.toRadians(shooterAngle))
                /Math.cos(Math.toRadians(shooterAngle)))*dHorizontal*dHorizontal
                /(dHorizontal*Math.tan(Math.toRadians(shooterAngle))-dVertical);
        if (vSquared < 0){ return -1;}
        return Math.sqrt(vSquared);
    }

    public double getShootingAngleErrorFromRPM(double rpm){
        // 1600rpm = -4.216304
        // 1200rpm = 0.537228
        //
        double a0 = 11.1772;
        double a1= -0.00919319;
        return a1*rpm + a0;
    }

   double scaleDrivePower(double dVal, double factor) {
      double[] scaleArray = {0.0, 0.14, 0.16, 0.18, 0.20, 0.22, 0.24, 0.27, 0.30, 0.34, 0.38, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00}; 

      // Get the corresponding index for the scaleDrivePower array.
      int index = (int) (dVal * 16.0);
      if (index < 0) {
         index = -index;
      } else if (index > 16) {
         index = 16;
      }

      double dScale = 0.0;
      if (dVal < 0) {
         dScale = -scaleArray[index];
      } else {
         dScale = scaleArray[index];
      }

      if( factor>0.0 && factor<=1.5) dScale *= factor;
      return dScale;
   }

   /// Low sensitivity drive mode for balancing and relic
   double scaleDrivePowerLowSensitivity(double dVal, double factor) {
      double[] scaleArray = { 0.0, 0.14, 0.16, 0.18, 0.20, 0.22, 0.24, 0.26, 0.28, 0.30, 0.32, 0.34, 0.36, 0.38, 0.40, 0.42, 0.44};  // Y17, with encoder

      // Get the corresponding index for the scaleDrivePower array.
      int index = (int) (dVal * 16.0);
      if (index < 0) {
         index = -index;
      } else if (index > 16) {
         index = 16;
      }

      double dScale = 0.0;
      if (dVal < 0) {
         dScale = -scaleArray[index];
      } else {
         dScale = scaleArray[index];
      }

      if( factor>0.0 && factor<=1.5) dScale *= factor;
      return dScale;
   }

   /// Low sensitivity drive mode 
   double scaleDrivePowerLowSensitivitySidewalk(double dVal, double factor) {
      double[] scaleArray = {0.0, 0.22, 0.24, 0.26, 0.28, 0.30, 0.32, 0.34, 0.36, 0.38, 0.40, 0.42, 0.44, 0.46, 0.48, 0.50, 0.52};  // Y20, TRMec

      // Get the corresponding index for the scaleDrivePower array.
      int index = (int) (dVal * 16.0);
      if (index < 0) {
         index = -index;
      } else if (index > 16) {
         index = 16;
      }

      double dScale = 0.0;
      if (dVal < 0) {
         dScale = -scaleArray[index];
      } else {
         dScale = scaleArray[index];
      }

      if( factor>0.0 && factor<=1.5) dScale *= factor;
      return dScale;
   }

   /// Scale the robot rotating power
   double scaleRotatePower(double dVal) {
      double[] scaleArray = {0.0, 0.35, 0.4, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.78, 0.80, 0.85, 0.90, 0.93, 0.97, 1.0};   // Y17, with encoder for Mecanum 6

      // Get the corresponding index for the scalePower array.
      int index = (int) (dVal * 16.0);
      if (index < 0) {
         index = -index;
      } else if (index > 16) {
         index = 16;
      }

      double dScale = 0.0;
      if (dVal < 0) {
         dScale = -scaleArray[index];
      } else {
         dScale = scaleArray[index];
      }

      double ENCODER_MAX_ROTATE_POWER = 1.0;
      dScale *= ENCODER_MAX_ROTATE_POWER;
      return dScale;
   }


}
