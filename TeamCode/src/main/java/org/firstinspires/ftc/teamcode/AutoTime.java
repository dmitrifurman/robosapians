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

import java.util.Objects;

import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.TeamColor;
/*
@Autonomous(name = "Advanced Auto w/ Functions", group = "Linear OpModes")
public class AutoTime extends LinearOpMode {

    private HardwareRobot robot = new HardwareRobot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting
        telemetry.addData("Status: ", "Initializing");


        robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.clearAll();
        telemetry.addData("Status: ", "Ready");
        telemetry.addData("Left Motor Position: ", robot.motor1.getCurrentPosition());
        telemetry.addData("Right Motor Position: ", robot.motor1.getCurrentPosition());


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.clearAll();
        telemetry.addData("Status: ", "Running");
        telemetry.addData("Left Motor Position: ", robot.motor1.getCurrentPosition());
        telemetry.addData("Right Motor Position: ", robot.motor1.getCurrentPosition());
        telemetry.addData("Belt Position: ", robot.motor5.getCurrentPosition());
        telemetry.update();


        runtime.reset();

        Move(0.5, 2.2, 0.5);

        Launch(2);

        Move(0.6, 3, 0.5);


        telemetry.addData("Status: ", "Complete");
        telemetry.update();
    }


    private void Move(double speed, double time, double pause)
            throws InterruptedException {

        robot.motor1.setPower(-speed);
        robot.motor2.setPower(-speed);

        while (opModeIsActive() && (runtime.seconds() < time)) {
            idle();
        }

        robot.motor1.setPower(0);
        robot.motor2.setPower(0);

        sleep((int)(pause * 1000));

        runtime.reset();

    }

    private void Launch(double balls)
            throws InterruptedException {

        robot.motor3.setPower(-1);
        robot.motor4.setPower(-1);

        for (int l = 1; l <= balls; l++) {

            while (opModeIsActive() && (runtime.seconds() < 1)) {
                idle();
            }

            robot.motor5.setPower(0.25);

            while ((robot.motor5.getCurrentPosition() < 600) && (robot.motor5.getCurrentPosition() > -600)) {
                idle();
            }

            robot.motor5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset belt encoder
            robot.motor5.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Reset mode to use encoder

            runtime.reset();

        }

        robot.motor3.setPower(0);
        robot.motor4.setPower(0);

    }


    public void sensorTest() {

        telemetry.addData("In Sensor Test", "NOW");
        telemetry.update();

        if (opModeIsActive()) {
            robot.color.enableLed(false);
            if (robot.color.red() >= 1 && robot.color.blue() == 0) {
                if (Objects.equals(TeamColor, "Red")) {
                    robot.btnPush.setPosition(Servo.MAX_POSITION);
                } else if (Objects.equals(TeamColor, "Blue")) {
                    robot.btnPush.setPosition(Servo.MIN_POSITION);
                }
            } else if (robot.color.blue() >= 1 && robot.color.red() == 0) {
                if (Objects.equals(TeamColor, "Red")) {
                    robot.btnPush.setPosition(Servo.MAX_POSITION);
                } else if (Objects.equals(TeamColor, "Blue")) {
                    robot.btnPush.setPosition(Servo.MIN_POSITION);
                }
            } else {
                robot.btnPush.setPosition(Servo.MAX_POSITION / 2);
            }
        }
    }

    public void gyroTurn(int angle, java.lang.String Direction, double time, double pause) throws InterruptedException {

        int gyroAngle = angle;

        robot.gyro.calibrate();
        while (robot.gyro.isCalibrating()) {
            idle();
        }

        if(Objects.equals(Direction, "Right")){
            gyroAngle = 360 - angle;
        }

        runtime.reset();

        while (robot.gyro.getHeading() != gyroAngle && (runtime.seconds() < time)) {
            if (Objects.equals(Direction, "Right")) {
                robot.motor1.setPower(0.25);
                robot.motor2.setPower(-0.25);
                telemetry.addData("Gyro heading: ", robot.gyro.getHeading());
            } else if (Objects.equals(Direction, "Left")) {
                robot.motor1.setPower(-0.25);
                robot.motor2.setPower(0.25);
                telemetry.addData("Gyro heading: ", robot.gyro.getHeading());
            }
        }

        robot.motor1.setPower(0);
        robot.motor2.setPower(0);

        sleep((int) (pause * 1000));
    }


}
*/