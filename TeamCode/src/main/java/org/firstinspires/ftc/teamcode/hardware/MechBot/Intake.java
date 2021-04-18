package org.firstinspires.ftc.teamcode.hardware.MechBot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

/**
 * FoundationHook spec:
 */
public class Intake extends Logger<Intake>  {

    final private CoreSystem core;

    private DcMotorEx intake1;
    private DcMotorEx intake2;

    private double IntakeSpeed = 1000;
    private double IntakePower = 0.8;

    private boolean isIntakeOn = false;

    public String getUniqueName() {
        return "intake";
    }

    /**
     * Hanging constructor
     */
    public Intake(CoreSystem core) {
        this.core = core;
    }

    public void reset(boolean Auto) {
        if (intake1 != null)
            init();
    }

    public void configure(Configuration configuration, boolean auto) {
        intake1 = configuration.getHardwareMap().get(DcMotorEx.class, "intake1");
        intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // intake1.setDirection(DcMotorSimple.Direction.REVERSE);
        // intake2 = configuration.getHardwareMap().get(DcMotorEx.class, "intake2");
        init();
      // configuration.register(this);
    }

    public void stop() {
        if (intake1!=null)
            intake1.setPower(0);
        if (intake2!=null)
            intake2.setPower(0);
        isIntakeOn = false;
    }

    public void init() {
        stop();
    }

    public void intakeOut(){
        if (intake1!=null)
            intake1.setPower(IntakePower);
        if (intake2!=null)
            intake2.setPower(IntakePower);
        isIntakeOn = true;
    }

    public void intakeIn(){
        if (intake1!=null)
            intake1.setPower(-IntakePower);
        if (intake2!=null)
            intake2.setPower(-IntakePower);
        isIntakeOn = true;
    }

    public void intakeOutBySpeed(){
        if (intake1!=null)
            intake1.setVelocity(IntakeSpeed);
        if (intake2!=null)
            intake2.setVelocity(IntakeSpeed);
        isIntakeOn = true;
    }

    public void intakeInBySpeed(){
        if (intake1!=null)
            intake1.setVelocity(-IntakeSpeed);
        if (intake2!=null)
            intake2.setVelocity(-IntakeSpeed);
        isIntakeOn = true;
    }

    public void intakeInAuto(){
        if(isIntakeOn)
            stop();
        else
            intakeIn();
    }

    public void intakeOutAuto(){
        if(isIntakeOn)
            stop();
        else
            intakeOut();
    }

    /**
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     * drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     * and servo position for each wheel
     */
    public void setupTelemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();

        if (intake1 != null) {
            line.addData(" | Intake1", new Func<String>() {
                @Override
                public String value() {
                    return String.format("(pow=%.2f/tps=%2.0f)\n", intake1.getPower(), intake1.getVelocity());
                }
            });
        }

        if (intake2 != null) {
            line.addData("Intake2", "pow=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return intake2.getPower();
                }
            });
        }
    }

}



