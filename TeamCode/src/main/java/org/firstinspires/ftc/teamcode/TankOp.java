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
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

public void turnGyroTarget(int angle, double power, int maxTime, char turnDirection) throws InterruptedException{

        long start = System.currentTimeMillis();
        long end = start + maxTime * 1000; // max time in seconds * 1000 ms per second

        angle -= 16;




        if (turnDirection =='r') {
            do {
                robot.motor2.setPower(power * -1);
                robot.motor1.setPower(power);
                telemetry.addData("Gyro heading", robot.gyro.getHeading());
                telemetry.update();

            }
            while (robot.gyro.getHeading() < angle && (System.currentTimeMillis() < end));

        } else if (turnDirection == 'l') {

            while (((robot.gyro.getHeading() > (360 - angle)) || (robot.gyro.getHeading() == 0)) && (System.currentTimeMillis() < end)){
                robot.motor1.setPower(power * -1);
                robot.motor2.setPower(power);
                telemetry.addData("Gyro heading", robot.gyro.getHeading());
                telemetry.update();

            }
        }

        robot.motor2.setPower(0); //turn both motors off
        robot.motor1.setPower(0);

        robot.gyro.calibrate();
        sleep(2000);


    }
*/

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Tank Op", group = "Linear Opmode")
public class TankOp extends OpMode {
    private final static double Arm_Min_Range = 0;
    private final static double Arm_Max_Range = 1;
    private final static double Extender_Max = 22000;
    private final static double Extender_Min = 0;
    private final static double BeltInterval = 600;
    private HardwareRobot robot = new HardwareRobot();
    private double driveMode = 0;
    private double armPosition = 0;
    private double armChange = 0.025;
    private double LEDmode = 0;
    private double beltMode = 0;
    private double collectMode = 0;
    private double extendMode = 0;

    public TankOp() {

    }

    @Override
    public void init() {
        robot.init(hardwareMap);

        // Sets Default Drive Mode
        driveMode = 0;

        // Sets button puah position
        robot.btnPush.setPosition(Servo.MAX_POSITION);

        // Servo Default Position
        armPosition = 0;
        robot.release1.setPosition(armPosition);
        robot.release2.setPosition(1-armPosition);

        // Sets Default Color Senor Mode
        LEDmode = 0;

        // Set Belt Mode
        beltMode = 0;

        // Set Collect Mode
        collectMode = 0;

        // Set Extend Mode
        extendMode = 0;

        // Sets Startng Robot Target Positions to Zero
        robot.motor5.setTargetPosition(0);
//        robot.motor7.setTargetPosition(0);

        // Reset belt encoder
        robot.motor5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    @Override
    public void loop() {

        feedback();
        drive();
        particleLaunch();
        particleCollector();
        servos();
//        extender();

    }

    private void feedback() {

        // Feedback Data
        telemetry.addData("Feedback Data for", "TankOp");
        telemetry.addData("Left Motor", robot.motor1.getPower());
        telemetry.addData("Right Motor", robot.motor2.getPower());
        telemetry.addData("Left Launch Power", (robot.motor3.getPower() * -1));
        telemetry.addData("Right Launch Power", (robot.motor4.getPower() * -1));
        telemetry.addData("Belt Speed", robot.motor5.getPower());
        telemetry.addData("Belt Pos", robot.motor5.getCurrentPosition());
        telemetry.addData("Collector Speed", robot.motor6.getPower());
//        telemetry.addData("Extend Speed", robot.motor7.getPower());
//        telemetry.addData("Extend Pos", robot.motor7.getCurrentPosition());
        telemetry.addData("Servos 1/2 Pos", robot.release1.getPosition());
        telemetry.addData("Button Pusher", robot.btnPush.getPosition());

    }

    private void colorSensor() {

        // Color Sensor Mode Changer
        if (LEDmode == 0) {
            robot.color.enableLed(false);
        } else if (LEDmode == 1) {
            robot.color.enableLed(true);
        }

        // LED Mode Updater
        if (gamepad2.left_bumper) {
            if (LEDmode == 0) {
                LEDmode = 2;
            } else if (LEDmode == 1) {
                LEDmode = 3;
            }
        }
        if (!gamepad2.left_bumper) {
            if (LEDmode == 2) {
                LEDmode = 1;
            } else if (LEDmode == 3) {
                LEDmode = 0;
            }
        }

    }

    private void  drive() {

        // Drive Code
        if(driveMode == 0) {
            robot.motor1.setPower(gamepad1.left_stick_y);
            robot.motor2.setPower(gamepad1.right_stick_y);
        }else if(driveMode == 1){
            robot.motor1.setPower(gamepad1.left_stick_y*0.1);
            robot.motor2.setPower(gamepad1.right_stick_y*0.1);
        }

        if(gamepad1.start){
            if(driveMode == 0){
                driveMode = 2;
            }else if(driveMode == 1){
                driveMode = 3;
            }
        }

        if(!gamepad1.start){
            if(driveMode == 2){
                driveMode = 1;
            }else if(driveMode == 3){
                driveMode = 0;
            }
        }

    }

    private void  particleLaunch() {

        // Launching Code
        robot.motor3.setPower(-1);
        robot.motor4.setPower(-1);

        // Belt Mode Update
        if (gamepad2.a) {
            beltMode = 1;
        }else if (gamepad2.b){
            beltMode = 2;
        }

        if (!gamepad2.a && !gamepad2.b) {
            if (beltMode == 1) {
                beltMode = 0;
                robot.motor5.setPower(-1);
            } else if (beltMode == 2) {
                beltMode = 0;
                robot.motor5.setPower(1);
            }
        }

        if(robot.motor5.getCurrentPosition() < BeltInterval*-1 || robot.motor5.getCurrentPosition() > BeltInterval){
            robot.motor5.setPower(0);
            robot.motor5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motor5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


    }

    private void  servos() {

        //Servo Position Updater
        if (gamepad1.a) {
            armPosition += armChange;
        }
        if (gamepad1.b) {
            armPosition -= armChange;
        }

        //Limits Servo Movement
        armPosition = Range.clip(armPosition, Arm_Min_Range, Arm_Max_Range);

        // Updates Servoes
        robot.release1.setPosition(armPosition);
        robot.release2.setPosition(1-armPosition);

    }

    private void  particleCollector() {

        // Collect Code
        if (collectMode == 0) {
            robot.motor6.setPower(0);
        } else if (collectMode == 1) {
            robot.motor6.setPower(1);
        } else if (collectMode == 2) {
            robot.motor6.setPower(-1);
        }

        // Collector Mode Updater
        if (gamepad2.x || gamepad2.y) {
            if (collectMode == 0 && gamepad2.x) {
                collectMode = 3;
            } else if (collectMode == 0 && gamepad2.y) {
                collectMode = 4;
            } else if (collectMode == 1 || collectMode == 2) {
                collectMode = 5;
            }
        }
        if (!gamepad2.x && !gamepad2.y) {
            if (collectMode == 3) {
                collectMode = 1;
            } else if (collectMode == 4) {
                collectMode = 2;
            } else if (collectMode == 5) {
                collectMode = 0;
            }
        }

    }

/*    private void  extender() {

        // Limits The Range of the Motors
        if(robot.motor7.getPower() < 0 && robot.motor7.getCurrentPosition() < Extender_Min){
            robot.motor7.setPower(1);
            extendMode = 0;
        }
        if(robot.motor7.getPower() == 1 && robot.motor7.getCurrentPosition() >= Extender_Min){
            robot.motor7.setPower(0);
        }
        if(robot.motor7.getPower() > 0 && robot.motor7.getCurrentPosition() > Extender_Max){
            robot.motor7.setPower(-1);
            extendMode = 0;
        }
        if(robot.motor7.getPower() == -1 && robot.motor7.getCurrentPosition() <= Extender_Min){
            robot.motor7.setPower(0);
        }

        // Extend Code
        if (extendMode == 0 && gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0) {
            robot.motor7.setPower(0);
        } else if (extendMode == 1 && robot.motor7.getCurrentPosition() <= Extender_Max) {
            robot.motor7.setPower(0.5);
        } else if (extendMode == 2 && robot.motor7.getCurrentPosition() >= Extender_Min) {
            robot.motor7.setPower(-0.5);
        } else if(gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0){
            robot.motor7.setPower(0);
        }

        // Automatic Extension
        if (gamepad2.left_bumper || gamepad2.right_bumper) {
            if (extendMode == 0 && gamepad2.left_bumper) {
                extendMode = 4;
            } else if (extendMode == 0 && gamepad2.right_bumper) {
                extendMode = 3;
            } else if (extendMode == 1 || extendMode == 2) {
                extendMode = 5;
            }
        }
        if (!gamepad2.left_bumper && !gamepad2.right_bumper) {
            if (extendMode == 3) {
                extendMode = 1;
            } else if (extendMode == 4) {
                extendMode = 2;
            } else if (extendMode == 5) {
                extendMode = 0;
            }
        }

        // Manual Extension
        if(robot.motor7.getCurrentPosition() <= Extender_Max && gamepad2.left_trigger > 0){
            extendMode = 0;
            robot.motor7.setPower(-1);
        }

        if(robot.motor7.getCurrentPosition() >= Extender_Min && gamepad2.right_trigger > 0){
            extendMode = 0;
            robot.motor7.setPower(1);
        }

    }
*/

    @Override
    public void stop() {
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
        robot.motor5.setPower(0);
        robot.motor6.setPower(0);
//        robot.motor7.setPower(0);
    }
}




