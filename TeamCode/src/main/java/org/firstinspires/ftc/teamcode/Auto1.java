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

//@Autonomous(name="Auto1", group="Linear OpModes")
public class Auto1 extends LinearOpMode {

    private HardwareRobot robot = new HardwareRobot();
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * (Math.PI));
    static final double     DRIVE_SPEED             = 0.75;
    static final double     TURN_SPEED              = 0.5;

    static final double rtTwo = Math.sqrt(2);

    double Time = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting
        telemetry.addData("Status:", "Initializing");
        telemetry.update();

        robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Time Left In Game", 30 - runtime.seconds());
        telemetry.addData("Auto1", "Starting at %7d :%7d",
                robot.motor1.getCurrentPosition(),
                robot.motor2.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        Move(-0.5, 1.7, 0.5);

        robot.motor3.setPower(-1);
        robot.motor4.setPower(-1);

        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            idle();
        }

        robot.motor5.setPower(0.25);


        while ((robot.motor5.getCurrentPosition() < 600) && (robot.motor5.getCurrentPosition() > -600)) {
            idle();
            //Do no task
        }

        robot.motor5.setPower(0);
        robot.motor5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset belt encoder
        robot.motor5.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Reset mode to use encoder


        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            idle();
        }

        robot.motor5.setPower(0.25); // MAY NEED TO CHANGE TO ADD NEGATIVE SIGN: WANT BELT TO SHOOT BALL IN FORWARD DIRECTION

        while ((robot.motor5.getCurrentPosition() < 600) && (robot.motor5.getCurrentPosition() > -600)) {
            idle();
            //Do no task
            //REMOVE ONE OF THE WHILE CONDITIONS BASED ON IF MOTOR 5 MOVES IN + OR - POWER
        }
        robot.motor5.setPower(0); // Stop belt motor
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
        robot.motor5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset belt encoder
        robot.motor5.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Reset mode to use encoder

        Move(-0.3, 6, 0.5);


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    // Speed: how fast, Time: how long in seconds, Pause: pause after move in seconds
    public void Move(double speed, double time, double pause) throws InterruptedException {

        telemetry.addData("Time Left in Move:", (time + (pause * 1000)) - runtime.seconds());
        telemetry.update();

        robot.motor1.setPower(speed);
        robot.motor2.setPower(speed);

        while (opModeIsActive() && (runtime.seconds() < time)) {
            idle();
        }

        robot.motor1.setPower(0);
        robot.motor2.setPower(0);

        sleep((int) (pause * 1000));

        runtime.reset();

    }


    public void sensorTest() {

        telemetry.addData("In Sensor Test", "NOW");
        telemetry.update();

        if (opModeIsActive()) {
            robot.color.enableLed(false);
            if (robot.color.red() >= 1 && robot.color.blue() == 0) {
                if (TeamColor == "Red") {
                    robot.btnPush.setPosition(Servo.MAX_POSITION);
                } else if (TeamColor == "Blue") {
                    robot.btnPush.setPosition(Servo.MIN_POSITION);
                }
            } else if (robot.color.blue() >= 1 && robot.color.red() == 0) {
                if (TeamColor == "Red") {
                    robot.btnPush.setPosition(Servo.MAX_POSITION);
                } else if (TeamColor == "Blue") {
                    robot.btnPush.setPosition(Servo.MIN_POSITION);
                }
            } else {/*Pick Neither*/}
        }

    }
}
