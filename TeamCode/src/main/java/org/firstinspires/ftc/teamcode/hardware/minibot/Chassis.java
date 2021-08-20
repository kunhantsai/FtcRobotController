package org.firstinspires.ftc.teamcode.hardware.minibot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Chassis {

    public DcMotor motorRight;
    public DcMotor motorLeft;

    public DistanceSensor rangeSensor;

    final Core core;

    public Chassis(Core c) {
        core = c;
    }

    BNO055IMU imu;
    Orientation angles;

    double target_heading = 0;

    // drive_power
    final static double DRIVE_RATIO = .9;
    boolean straight_mode = true; // if drive_power is used with encoders,true. else false

    // drive_distance
    final static int ONE_ROTATION = 1120; // for AndyMark-40 motor encoder one rotation
    final static double INCHES_PER_ROTATION = 12.57; // inches per chassis motor rotation based on 16/24 gear ratio
    double leftPower = 0; // tracks motor power across functions
    double rightPower = 0;
    int leftCount = 0; // motor target counter
    int rightCount = 0;

    // rotate_left & rotate_right
    final static double WHEEL_TURN_COMPLETE = 6.63;  // number of wheel turns to get chassis 360-degree setPosition
    final static double IMU_ROTATION_RATIO_L = 0.80; // 0.84; // ratio of IMU Sensor Left setPosition to prevent overshooting the setPosition.
    final static double IMU_ROTATION_RATIO_R = 0.80; // 0.84; // ratio of IMU Sensor Right setPosition to prevent overshooting the setPosition.


    public void init(HardwareMap hwMap) {
        motorLeft = hwMap.dcMotor.get("left_drive"); // should be motorLeft, not left_drive
        motorRight = hwMap.dcMotor.get("right_drive");

        // rangeSensor = hwMap.get(DistanceSensor.class, "rangeSensor");


        // initializes IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setDirection(DcMotor.Direction.FORWARD);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /**
     * shows motor power in telemetry
     * @param telemetry this.telemetry
     */
    public void show_diagnostic(Telemetry telemetry) {
        if (rangeSensor!=null)
            telemetry.addData("range", rangeSensor.getDistance(DistanceUnit.CM));
        if (motorLeft!=null)
            telemetry.addData("motorLeft:", motorLeft.getPower());
        if (motorRight!=null)
            telemetry.addData("motorRight:", motorRight.getPower());
    }

    /**
     * sets motors to left and right powers, corrects heading
     *
     * @param lp sets left power
     * @param rp sets right power
     */
    public void drive_power(double lp, double rp) {
        if (straight_mode) {
            double cur_heading = imu_heading();
            if (cur_heading - target_heading > 0.7) { // crook to left,  slow down right motor
                if (rp > 0) rp *= DRIVE_RATIO;
                else lp *= DRIVE_RATIO;
            } else if (cur_heading - target_heading < -0.7) { // crook to right, slow down left motor
                if (lp > 0) lp *= DRIVE_RATIO;
                else rp *= DRIVE_RATIO;
            }
        }
        if (Math.abs(rp) > 0.3 && Math.abs(lp) > 0.3) {
            motorLeft.setPower(lp * 1.0);
            motorRight.setPower(rp * 1.0);
        } else {
            motorLeft.setPower(lp);
            motorRight.setPower(rp);
        }
    }

    /***
     * Pivots robot counterclockwise using encoders and IMU
     * @param power speed of rotation
     * @param degree target degree from current orientation
     * @throws InterruptedException
     */
    public void rotate_left(double power, double degree) throws InterruptedException {
        double adjust_degree_navx = IMU_ROTATION_RATIO_L * (double) degree;
        double current_pos = 0;
        boolean heading_cross_zero = false;
        ElapsedTime runtime = new ElapsedTime();
        reset_chassis();
        //set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS)
        int leftEncode = motorLeft.getCurrentPosition();
        int rightEncode = motorRight.getCurrentPosition();
        leftCount = (int) (-ONE_ROTATION * WHEEL_TURN_COMPLETE * degree / 720.0);
        rightCount = (int) (ONE_ROTATION * WHEEL_TURN_COMPLETE * degree / 720.0);

        leftPower = (float) -power;
        rightPower = (float) power;

        leftCount += leftEncode;
        rightCount += rightEncode;


        current_pos = imu_heading();
        target_heading = current_pos + adjust_degree_navx;
        if (target_heading >= 180) {
            target_heading -= 360;
            heading_cross_zero = true;
        }
        if (heading_cross_zero && (current_pos >= 0)) {
            current_pos -= 360;
        }
        //DbgLog.msg(String.format("imu Left Turn curr/target pos = %.2f/%.2f.", current_pos, target_heading));
        while ((current_pos <= target_heading) && (runtime.seconds() < 5.0)) {
            current_pos = imu_heading();
            if (heading_cross_zero && (current_pos >= 0)) {
                current_pos -= 360;
            }
            drive_distance(leftPower, rightPower);
        }
    }

    /***
     * Pivots robot counterclockwise using encoders and IMU
     * @param power speed of rotation
     * @param degree target degree from current orientation
     * @throws InterruptedException
     */
    public void rotate_right(double power, double degree) throws InterruptedException {
        double adjust_degree_navx = IMU_ROTATION_RATIO_R * degree;
        double current_pos = 0;
        boolean heading_cross_zero = false;
        ElapsedTime runtime = new ElapsedTime();
        reset_chassis();
        int leftEncode = motorLeft.getCurrentPosition();
        int rightEncode = motorRight.getCurrentPosition();
        leftCount = (int) (ONE_ROTATION * WHEEL_TURN_COMPLETE * degree / 720.0);
        rightCount = (int) (-ONE_ROTATION * WHEEL_TURN_COMPLETE * degree / 720.0);

        leftPower = (float) power;
        rightPower = (float) -power;

        leftCount += leftEncode;
        rightCount += rightEncode;
        // DbgLog.msg(String.format("imu Right Turn %.2f degree with %.2f power.", degree, power));
        current_pos = imu_heading();
        target_heading = current_pos - adjust_degree_navx;
        if (target_heading <= -180) {
            target_heading += 360;
            heading_cross_zero = true;
        }
        if (heading_cross_zero && (current_pos <= 0)) {
            current_pos += 360;
        }
        while ((current_pos >= target_heading) && (runtime.seconds() < 4.0)) {
            current_pos = imu_heading();
            // DbgLog.msg(String.format("imu current/target heading = %.2f/%.2f",current_pos,target_heading));

            if (heading_cross_zero && (current_pos <= 0)) {
                current_pos += 360;
            }
            drive_distance(leftPower, rightPower);
        }
    }

    public void drive_time_without_imu(double power, long time) {
        motorLeft.setPower(power);
        motorRight.setPower(power);
        long timeStart = System.currentTimeMillis();
        while (System.currentTimeMillis() - timeStart < time) {
            core.yield();
        }
        stop_chassis();
    }


    /***
     *
     drives straight a given distance (in inches) using encoders
     * @param power
     * @param in
     * @throws InterruptedException
     */

    public void drive_distance(double power, double in) throws InterruptedException {
        target_heading = imu_heading();
        double rotations = in / INCHES_PER_ROTATION;

        straight_mode = true;
        reset_chassis();

        int leftEncode = motorLeft.getCurrentPosition();
        int rightEncode = motorRight.getCurrentPosition();

        leftCount = (int) (ONE_ROTATION * rotations);
        rightCount = (int) (ONE_ROTATION * rotations);
        leftPower = rightPower = (float) power;
        if (power < 0) { // backwards
            leftCount = leftEncode - leftCount;
            rightCount = rightEncode - rightCount;
        } else {
            leftCount += leftEncode;
            rightCount += rightEncode;
        }
        run_until_encoder(leftCount, leftPower, rightCount, rightPower);
        straight_mode = false;
    }

    void run_until_encoder(int leftCnt, double leftPower, int rightCnt, double rightPower) throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();
        drive_power(leftPower, rightPower);
        runtime.reset();
        while (!have_drive_encoders_reached(leftCnt, rightCnt) && (runtime.seconds() < 5)) {
            drive_power(leftPower, rightPower);

            //**************The line*****************
            core.yield();
            //***************************************
        }
        stop_chassis();
    }

    // indicate whether the drive_power motors' encoders have reached specified values.
    boolean have_drive_encoders_reached(double leftCnt, double rightCnt) {
        boolean l_return = false;
        if (has_left_drive_encoder_reached(leftCnt) && has_right_drive_encoder_reached(rightCnt)) {
            l_return = true;
        } else if (has_left_drive_encoder_reached(leftCnt)) { // shift target encoder value from right to left
            double diff = Math.abs(rightCnt - motorRight.getCurrentPosition()) / 2;
            if (leftPower < 0) {
                leftCount -= diff;
            } else {
                leftCount += diff;
            }
            if (rightPower < 0) {
                rightCount += diff;
            } else {
                rightCount -= diff;
            }
        } else if (has_right_drive_encoder_reached(rightCnt)) { // shift target encoder value from left to right
            double diff = Math.abs(leftCnt - motorLeft.getCurrentPosition()) / 2;
            if (rightPower < 0) {
                rightCount -= diff;
            } else {
                rightCount += diff;
            }
            if (leftPower < 0) {
                leftCount += diff;
            } else {
                leftCount -= diff;
            }
        }
        return l_return;
    }

    boolean has_left_drive_encoder_reached(double leftCnt) {
        if (leftPower < 0) {
            return (motorLeft.getCurrentPosition() <= leftCnt);
        } else {
            return (motorLeft.getCurrentPosition() >= leftCnt);
        }
    }

    boolean has_right_drive_encoder_reached(double rightCnt) {
        if (rightPower < 0) {
            return (motorRight.getCurrentPosition() <= rightCnt);
        } else {
            return (motorRight.getCurrentPosition() >= rightCnt);
        }

    }

    double imu_heading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void stop_chassis() {
        motorLeft.setPower(0);
        motorRight.setPower(0);
        //encMotor.setPower(0);
    }

    void reset_chassis() {
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftCount = 0;
        rightCount = 0;
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
