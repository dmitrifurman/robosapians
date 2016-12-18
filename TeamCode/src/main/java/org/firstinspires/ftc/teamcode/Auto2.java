/* Copyright (c) 2016 ROBOSAPIANS

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are not permitted (subject to the exceptions in the disclaimer below)
Exceptions are provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robosapians nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.TeamColor;

//@Autonomous(name="Auto2", group="Linear OpModes")
public class Auto2 extends LinearOpMode {

    private HardwareRobot robot = new HardwareRobot();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * (Math.PI));
    static final double DRIVE_SPEED = 0.8;
    static final double TURN_SPEED = 0.5;

    static final double rtTwo = Math.sqrt(2);

    @Override
    public void runOpMode() throws InterruptedException {

        //Hi
        //Map all components to Hardware Robot file
        robot.init(hardwareMap);

        // Telemetry Init
        telemetry.addData("Status", "Initializing");    //
        telemetry.update();

        //Reset any prior encoder values
        robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        //Set Run Mode to use encoder
        robot.motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry for Current Status
        telemetry.addData("Starting at", "%7d : %7d", robot.motor1.getCurrentPosition(), robot.motor2.getCurrentPosition());
        telemetry.update();

        // Wait for start
        waitForStart();

        //Remember: Reverse movement is obtained by setting a negative distance (not speed)

        // Set in the exact middle of 3rd tile from left (if on RED team)
        // Set in the exact middle of 3rd tile from right (if on BLUE team)
        encoderDrive(DRIVE_SPEED, -37, -37, 11.0); // Drive forward 37 inches
        // Launching Code
        robot.motor3.setPower(-1);
        robot.motor4.setPower(-1);


        robot.motor5.setPower(0.25); // MAY NEED TO CHANGE TO ADD NEGATIVE SIGN: WANT BELT TO SHOOT BALL IN FORWARD DIRECTION


        while ((robot.motor5.getCurrentPosition() < 1200) && (robot.motor5.getCurrentPosition() > -1200)) {
            //Do no task
            //REMOVE ONE OF THE WHILE CONDITIONS BASED ON IF MOTOR 5 MOVES IN + OR - POWER
        }
        robot.motor5.setPower(0); // Stop belt motor
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
        robot.motor5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset belt encoder
        robot.motor5.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Reset mode to use encoder

        encoderDrive(DRIVE_SPEED, 14, 14, 6.0); // Drive forward 14 inches


        // Turns left (if on RED team)
        // Turns right (if on BLUE team)
        if (TeamColor == "Red") {
            encoderDrive(TURN_SPEED, -9 * rtTwo, 9 * rtTwo, 9.0); // Turns left
        } else {
            encoderDrive(TURN_SPEED, 9 * rtTwo, -9 * rtTwo, 9.0); // Turns right
        }

        encoderDrive(DRIVE_SPEED, 39, 39, 8.0); // Drives forward to total 3in away from beacon

        encoderDrive(0.4, 1.5, 1.5, 1.0);
        beaconPush();
        sleep(800);
        encoderDrive(0.4, 1.5, 1.5, 1.0);
        sleep(300);
        encoderDrive(0.4, -1.5, -2.5, 1.0);
        sleep(300);
        encoderDrive(0.4, 1.5, 2.5, 1.0);
        encoderDrive(0.5, -12, -12, 1.0); // Now we are 9in away from the beacon touching position

        // Turns right (if on RED team)
        // Turns left (if on BLUE team)
        if (TeamColor == "Red") {
            encoderDrive(TURN_SPEED, 9 * rtTwo, -9 * rtTwo, 9.0); // Turns right
        } else {
            encoderDrive(TURN_SPEED, -9 * rtTwo, 9 * rtTwo, 9.0); // Turn left
        }

        // Move to the next beacon
        encoderDrive(DRIVE_SPEED, 48, 48, 5.0);

        // Turns left (if on RED team)
        // Turns right (if on BLUE team)
        if (TeamColor == "Red") {
            encoderDrive(TURN_SPEED, -9 * rtTwo, 9 * rtTwo, 9.0); // Turns left
        } else {
            encoderDrive(TURN_SPEED, 9 * rtTwo, -9 * rtTwo, 9.0); // Turns right
        }

        // Next section needs to start 3 in. away and centered to the beacon
        encoderDrive(0.4, 6, 6, 1.0); // Now we are 3in away from the beacon touching position
        beaconPush();
        sleep(800);
        encoderDrive(0.4, 1.5, 1.5, 1.0);
        sleep(300);
        encoderDrive(0.4, -1.5, -2.5, 1.0);
        sleep(300);
        encoderDrive(0.4, 1.5, 2.5, 1.0);
        encoderDrive(0.5, -15, -15, 1.0); // Now we are 15 in away from field wall

        // Optional last step: move on ramp
        //encoderDrive(DRIVE_SPEED, (18*rtTwo), (18*rtTwo), 3.0);
        //encoderDrive(TURN_SPEED,   38.0, -38.0, 4.0);
        //encoderDrive(DRIVE_SPEED, (30*rtTwo), (30*rtTwo), 4.0); */

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.motor1.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.motor2.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.motor1.setTargetPosition(newLeftTarget);
            robot.motor2.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Reset the timeout time and start motion.
            runtime.reset();
            robot.motor1.setPower(speed);
            robot.motor2.setPower(speed);

            // Keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motor1.isBusy() && robot.motor2.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.motor1.getCurrentPosition(),
                        robot.motor2.getCurrentPosition());
                telemetry.update();

                if (leftInches == (90 * rtTwo) && robot.motor1.getCurrentPosition() >= (12 * rtTwo) + 23 && robot.motor1.getCurrentPosition() <= (60 * rtTwo) + 23) {
                    robot.motor3.setPower(-1);
                    robot.motor4.setPower(-1);
                }
                if (leftInches == (90 * rtTwo) && robot.motor1.getCurrentPosition() >= (18 * rtTwo) + 23 && robot.motor1.getCurrentPosition() <= (60 * rtTwo) + 23) {
                    robot.motor5.setPower(-0.25);
                }
                if (leftInches == (90 * rtTwo) && robot.motor1.getCurrentPosition() >= (60 * rtTwo) + 23) {
                    robot.motor3.setPower(0);
                    robot.motor4.setPower(0);
                    robot.motor5.setPower(0);
                }

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.motor1.setPower(0);
            robot.motor2.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Short pause after each move for testing
            sleep(500);
        }
    }

    public void turnRightGyroTarget(int angle, double power, int maxTimeInSec) {

        robot.gyro.calibrate();
        while (robot.gyro.isCalibrating()) {
            // Nothing
        }

        long start = System.currentTimeMillis();
        long end = start + maxTimeInSec; // max time in milliseconds * 1000 ms per second

        while (robot.gyro.getHeading() != angle && (System.currentTimeMillis() < end)) {
            robot.motor1.setPower(power * -1);
            robot.motor2.setPower(power);
            telemetry.addData("Gyro heading", robot.gyro.getHeading());
        }

        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
    }

    public void beaconPush() {

        robot.color.enableLed(false);
        if (robot.color.red() >= 1 && robot.color.blue() < 1) {
            if (TeamColor == "Red") {
                robot.btnPush.setPosition(Servo.MIN_POSITION);
            } else {
                robot.btnPush.setPosition(Servo.MAX_POSITION);
            }
        } else if (robot.color.blue() >= 1 && robot.color.red() < 1) {
            if (TeamColor == "Red") {
                robot.btnPush.setPosition(Servo.MIN_POSITION);
            } else {
                robot.btnPush.setPosition(Servo.MAX_POSITION);
            }
            // Prevents tha robot from choosing the wrong color for the beacon
        } else {
            robot.btnPush.setPosition(Servo.MAX_POSITION / 2);
        }

    }

}

