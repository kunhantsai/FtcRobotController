package org.firstinspires.ftc.teamcode.components.odometry;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.components.CombinedOrientationSensor;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 */
public class OdometryGlobalCoordinatePosition implements Runnable{
    //Odometry wheels
    private DcMotorEx verticalEncoderLeft, verticalEncoderRight, horizontalEncoder;
    private CombinedOrientationSensor orientationSensor;
    //Thead run condition
    private boolean isRunning = true;
    private boolean useIMU = false;
    //Position variables used for storage and calculations
    double initRadians = 0;
    double verticalRightEncoderWheelPosition = 0, verticalLeftEncoderWheelPosition = 0, normalEncoderWheelPosition = 0,  changeInRobotOrientation = 0;
    private double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;
    private double xSpeedDegree = 0, ySpeedDegree = 0;
    private double curSpeed = 0, prevSpeed = 0;
    private double curTime = 0, prevTime = 0;
    private double maxSpeed = 0;
    private double curAcc = 0;
    private double maxAcc = 0, minAcc = 0;
    private double xSpeedLogs[] = {0,0,0,0,0};
    private double ySpeedLogs[] = {0,0,0,0,0};
    private int count=0;
    private double previousVerticalRightEncoderWheelPosition = 0, previousVerticalLeftEncoderWheelPosition = 0, prevNormalEncoderWheelPosition = 0;
    private double Y_COUNTS_PER_INCH = 307.7; // should be synchronized with MechChassis.java
    private double X_COUNTS_PER_INCH = 305.6; // should be synchronized with MechChassis.java
    // private double netRotations; // clockwise rotations in degrees
    private double rRotations; // clockwise rotations in degrees
    private double lRotations; // counter-clockwise rotations in degrees
    private double prevHeading;  // previous heading


    //Algorithm constants
    //private double robotEncoderWheelDistance = 15.20435 * Y_COUNTS_PER_INCH;
    // private double robotEncoderWheelDistance = 15.4317822 * Y_COUNTS_PER_INCH;
    // private double horizontalEncoderTickPerDegreeOffset = -86.84834;
    private double robotEncoderWheelDistance = 14.5 * Y_COUNTS_PER_INCH;
    private double horizontalEncoderTickPerDegreeOffset = 84.26;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    //Files to access the algorithm constants
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    private double verticalLeftEncoderPositionMultiplier = 1;  // 0.8680555556;;
    private double verticalRightEncoderPositionMultiplier = 1; // 1.152;
    private double normalEncoderPositionMultiplier = 1;
    private int GPSVersion = 1; // version 1 - Wizard Odometry
                                // version 2 - Beta Odometry
    public void set_counts_per_inch(double x, double y) {
        X_COUNTS_PER_INCH = x;
        Y_COUNTS_PER_INCH = y;
    }
    public double getlRotations() { return lRotations; }
    public double getrRotations() { return rRotations; }
    public double getCurSpeed() { return curSpeed; }
    public double getMaxSpeed() { return maxSpeed; }
    public double getCurAcc() { return curAcc; }
    public double getMinAcc() { return minAcc; }
    public double getMaxAcc() { return maxAcc; }


    public double rotationCorrection() { // degrees to be corrected due to rotation error
        return (getrRotations() * 1.45 + getlRotations() * 1.1) / 360.0;
    }


    public void set_orientationSensor(CombinedOrientationSensor val) {
        orientationSensor = val;
        if (orientationSensor!=null) {
            prevHeading = orientationSensor.getHeading();
        }
        rRotations = lRotations = 0;
    }

    /**
     * Constructor for GlobalCoordinatePosition Thread
     * @param verticalEncoderLeft left odometry encoder, facing the vertical direction
     * @param verticalEncoderRight right odometry encoder, facing the vertical direction
     * @param horizontalEncoder horizontal odometry encoder, perpendicular to the other two odometry encoder wheels
     * @param threadSleepDelay delay in milliseconds for the GlobalPositionUpdate thread (50-75 milliseconds is suggested)
     */
    public OdometryGlobalCoordinatePosition(DcMotorEx verticalEncoderLeft, DcMotorEx verticalEncoderRight, DcMotorEx horizontalEncoder, int threadSleepDelay, int version){
        this.verticalEncoderLeft = verticalEncoderLeft;
        this.verticalEncoderRight = verticalEncoderRight;
        this.horizontalEncoder = horizontalEncoder;
        sleepTime = threadSleepDelay;
        GPSVersion = version;

//        if (wheelBaseSeparationFile.exists()) {
//            robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
//        }
        if (horizontalTickOffsetFile.exists()) {
            this.horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
        }
        if(GPSVersion == 2) {
            Y_COUNTS_PER_INCH = 334.6;
            X_COUNTS_PER_INCH = 330.56;
            robotEncoderWheelDistance = 14.75 * Y_COUNTS_PER_INCH;
            horizontalEncoderTickPerDegreeOffset = 60;
            useIMU=false;
        }
    }

    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    private void globalCoordinatePositionUpdate(){
        curSpeed = Math.hypot((Math.abs(verticalEncoderLeft.getVelocity(AngleUnit.DEGREES)) + Math.abs(verticalEncoderRight.getVelocity(AngleUnit.DEGREES)))/2.0,
                horizontalEncoder.getVelocity(AngleUnit.DEGREES));
        curTime = System.currentTimeMillis();
        if (curTime-prevTime!=0)
           curAcc = (curSpeed-prevSpeed)/(curTime-prevTime);
        if (curSpeed>maxSpeed)
            maxSpeed = curSpeed;
        if (curAcc>maxAcc)
            maxAcc = curAcc;
        if (curAcc<minAcc)
            minAcc = curAcc;
        //Get Current Positions
        verticalLeftEncoderWheelPosition = (verticalEncoderLeft.getCurrentPosition()* verticalLeftEncoderPositionMultiplier);
        verticalRightEncoderWheelPosition = (verticalEncoderRight.getCurrentPosition() * verticalRightEncoderPositionMultiplier);
        normalEncoderWheelPosition = (horizontalEncoder.getCurrentPosition()*normalEncoderPositionMultiplier);

        double leftChange = verticalLeftEncoderWheelPosition - previousVerticalLeftEncoderWheelPosition;
        double rightChange = verticalRightEncoderWheelPosition - previousVerticalRightEncoderWheelPosition;

        //Calculate Angle
        changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance);
        // changeInRobotOrientation = Math.signum(leftChange - rightChange) * Math.acos(1  - (leftChange - rightChange)*(leftChange - rightChange)/ robotEncoderWheelDistance/ robotEncoderWheelDistance / 2);
        // Replace angle calculation by imu
        if ((orientationSensor!=null) && useIMU) { // robot is almost stop
            double curHeading = orientationSensor.getHeading();
            robotOrientationRadians = Math.toRadians(curHeading) + initRadians;
            // calculate robot netRotations
            double curRotation = curHeading-prevHeading;
            if (curRotation!=0) {
                if (curHeading * prevHeading < -1000) { // cross the +180/-180 gap
                    if (curHeading < 0) {
                        curRotation += 360;
                    } else {
                        curRotation -= 360;
                    }
                }
                if (curRotation > 0)
                    rRotations += curRotation;
                else
                    lRotations += curRotation;
                prevHeading = curHeading;
            }
        } else {
            robotOrientationRadians = ((robotOrientationRadians + changeInRobotOrientation));
        }
        //Get the components of the motion
        double rawHorizontalChange = normalEncoderWheelPosition - prevNormalEncoderWheelPosition;
        double horizontalChange = rawHorizontalChange - (changeInRobotOrientation*horizontalEncoderTickPerDegreeOffset);

        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange;

        //Calculate and update the position values
        robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + (p*Math.sin(robotOrientationRadians) + n*Math.cos(robotOrientationRadians));
        robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + (p*Math.cos(robotOrientationRadians) - n*Math.sin(robotOrientationRadians));

        previousVerticalLeftEncoderWheelPosition = verticalLeftEncoderWheelPosition;
        previousVerticalRightEncoderWheelPosition = verticalRightEncoderWheelPosition;
        prevNormalEncoderWheelPosition = normalEncoderWheelPosition;
        ySpeedLogs[(count%5)]=(Math.abs(verticalEncoderLeft.getVelocity(AngleUnit.DEGREES)) + Math.abs(verticalEncoderRight.getVelocity(AngleUnit.DEGREES))) / 2;
        xSpeedLogs[(count%5)]=Math.abs(horizontalEncoder.getVelocity(AngleUnit.DEGREES));
        if (count > 4) {
            ySpeedDegree = (ySpeedLogs[0]+ySpeedLogs[1]+ySpeedLogs[2]+ySpeedLogs[3]+ySpeedLogs[4])/5.0;
            xSpeedDegree = (xSpeedLogs[0]+xSpeedLogs[1]+xSpeedLogs[2]+xSpeedLogs[3]+xSpeedLogs[4])/5.0;
//            if ((ySpeedDegree+xSpeedDegree<1) && useIMU) { // robot is almost stop
//                robotOrientationRadians = Math.toRadians(orientationSensor.getHeading())+initRadians;
//            }
        }


        prevTime = curTime;
        prevSpeed = curSpeed;
        count = (count+1)%10000;

    }
    public void correctAngleUsingIMU(){
        if (orientationSensor!=null) {
            double curHeading = orientationSensor.getHeading();
            robotOrientationRadians = Math.toRadians(curHeading) + initRadians;
        }
    }

    /**
     * Returns the robot's global x coordinate
     * @return global x coordinate
     */
    public double returnXCoordinate(){ return robotGlobalXCoordinatePosition; }

    /**
     * Returns the robot's global y coordinate
     * @return global y coordinate
     */
    public double returnYCoordinate(){ return robotGlobalYCoordinatePosition; }

    /**
     * Returns the robot's global orientation
     * @return global orientation, in degrees
     */
    public double returnOrientation(){ return Math.toDegrees(robotOrientationRadians) % 360; }
    public void changeOrientation(double newOrientationDegrees){
        robotOrientationRadians = Math.toRadians(newOrientationDegrees);
    }
    public void set_init_pos(double x, double y, double degree) {
        robotOrientationRadians = Math.toRadians(degree);
        initRadians = robotOrientationRadians;
        robotGlobalXCoordinatePosition = x;
        robotGlobalYCoordinatePosition = y;
    }

    public void correctHeading(double degree) {
        initRadians = Math.toRadians(degree);
    }

    public void set_x_pos(double x_cm) {
//        horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        horizontalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        verticalEncoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        verticalEncoderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        verticalEncoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        verticalEncoderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robotOrientationRadians = Math.toRadians(0);
//        initRadians = robotOrientationRadians;
//        verticalRightEncoderWheelPosition = verticalLeftEncoderWheelPosition = normalEncoderWheelPosition = changeInRobotOrientation = 0;
//        previousVerticalRightEncoderWheelPosition = previousVerticalLeftEncoderWheelPosition = prevNormalEncoderWheelPosition = 0;
        robotGlobalXCoordinatePosition = x_cm * X_COUNTS_PER_INCH / 2.54;
    }

    public void set_y_pos(double y) {
        robotGlobalYCoordinatePosition = y;
    }
    public double leftYEncoder() { return verticalLeftEncoderWheelPosition; }
    public double rightYEncoder() { return verticalRightEncoderWheelPosition; }
    public double XEncoder() { return normalEncoderWheelPosition; }

    public double getXSpeedDegree() { return xSpeedDegree; }
    public double getYSpeedDegree() { return ySpeedDegree; }


    /**
     * Stops the position update thread
     */
    public void stop(){ isRunning = false; }

    public void reverseLeftEncoder(){
        if(verticalLeftEncoderPositionMultiplier == 1){
            verticalLeftEncoderPositionMultiplier = -1;
        }else{
            verticalLeftEncoderPositionMultiplier = 1;
        }
    }

    public void reverseRightEncoder(){
        if(verticalRightEncoderPositionMultiplier == 1){
            verticalRightEncoderPositionMultiplier = -1;
        }else{
            verticalRightEncoderPositionMultiplier = 1;
        }
    }

    public void reverseNormalEncoder(){
        if(normalEncoderPositionMultiplier == 1){
            normalEncoderPositionMultiplier = -1;
        }else{
            normalEncoderPositionMultiplier = 1;
        }
    }

    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            globalCoordinatePositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
