/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.hardware.minibot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@TeleOp(name="MiniBot: Teleop", group="MiniBot")
public class MiniBotTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    MiniRobot robot           = new MiniRobot();   // Use a Pushbot's hardware
    static double joy_threshold = 0.03;                                                          // could also use HardwarePushbotMatrix class.

    @Override
    public void runOpMode() {
        double left=0;
        double right=0;
        double max;
        double prev_right_y=0;
        double prev_left_y=0;
        double speedscale = 0.5;
        int left_event_counter =0;
        int right_event_counter =0;
        int loop_count = 0;
        int delay_scale = 100;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        double power_steps [] = {0.0, 0.5, 0.7, 0.85, 0.95, 1.0};
        int n_steps = 6;
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // if (Math.abs(prev_left_y - gamepad1.left_stick_y)>0) {
            if (left==0.0 && Math.abs(gamepad1.left_stick_y)>joy_threshold) {
                left_event_counter = 1;
            } else if (left>0.1 && Math.abs(gamepad1.left_stick_y)<joy_threshold) {
                left_event_counter = 5;
            } else {
                if (Math.abs(gamepad1.left_stick_y)>joy_threshold) {
                    if ((left_event_counter < n_steps-1) && (loop_count%delay_scale==4)) {
                        left_event_counter++;
                    }
                } else {
                    if ((left_event_counter > 0)&& (loop_count%delay_scale==4)) {
                        left_event_counter--;
                    }
                }
            }
            // if (Math.abs(prev_right_y - gamepad1.right_stick_y)>0) {
            if (right==0.0 && Math.abs(gamepad1.right_stick_y)>joy_threshold) {
                right_event_counter = 1;
            } else if (right>0.1 && Math.abs(gamepad1.right_stick_y)<joy_threshold) {
                right_event_counter = 5;
            } else {
                if (Math.abs(gamepad1.right_stick_y)>joy_threshold) {
                    if ((right_event_counter < n_steps-1) && (loop_count%delay_scale==4)) {
                        right_event_counter++;
                    }
                } else {
                    if ((right_event_counter > 0)&& (loop_count%delay_scale==4)) {
                        right_event_counter--;
                    }
                }
            }
            //left_event_counter = right_event_counter = 5;
            left = -gamepad1.left_stick_y*power_steps[left_event_counter];
            right = -gamepad1.right_stick_y*power_steps[right_event_counter];
            if ((loop_count%5)==1) {
                prev_right_y = -gamepad1.right_stick_y;
                prev_left_y = -gamepad1.left_stick_y;
            }

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            left *= speedscale;
            right *= speedscale;
            robot.chassis.motorLeft.setPower(left);
            robot.chassis.motorRight.setPower(right);
            if (gamepad1.y && (speedscale<1.0)) {
                speedscale += 0.05;
                sleep(80);
            }
            else if (gamepad1.a && (speedscale>0.2)) {
                speedscale -= 0.05;
                sleep(80);
            }
//            if (gamepad1.left_trigger>0.1) {
//                robot.l_kicker_down();
//            } else if (gamepad1.left_bumper) {
//                robot.l_kicker_up();
//            }
//            if (gamepad1.right_trigger>0.1) {
//                robot.r_kicker_down();
//            } else if (gamepad1.right_bumper) {
//                robot.r_kicker_up();
//            }
            telemetry.addData("left/right motor  =", "%.2f/%.2f", left,right);
            // telemetry.addData("prev left/right y  =", "%.2f/%.2f", prev_left_y,prev_right_y);
            telemetry.addData("left/right counter =", "%d/%d", left_event_counter, right_event_counter);
            telemetry.addData("speed scale =", "%.2f", speedscale);
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            // robot.waitForTick(40);
            loop_count++;
        }
    }
}
