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


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TankOp", group = "Linear Opmode")
public class TankOp extends OpMode {
    private HardwareRobot robot = new HardwareRobot();
    private final static double Arm_Min_Range = 0;
    private final static double Arm_Max_Range = 1;
    private final static double Extender_Min = -23000;
    private final static double Extender_Max = 0;
    private final static double BeltInterval = 600;

    private double armPosition = 0;
    private double armChange = 0.025;
    private double LEDmode = 0;
    private double beltMode = 0;
    private double collectMode = 0;
    private double extendMode = 0;

    private double beltTarget = 0;
    private double beltChange = 5000;


    public TankOp() {

    }

    @Override
    public void init() {
        robot.init(hardwareMap);
        // Servo Default Position
        armPosition = 0;

        // Sets Default Color Senor Mode
        LEDmode = 0;

        // Set Belt Mode
        beltMode = 0;

        // Set Collect Mode
        collectMode = 0;

        // Ser Extend Mode
        extendMode = 0;

        // Sets Startng Robot Extender Position to Zero
        robot.motor5.setTargetPosition(0);
        robot.motor7.setTargetPosition(-1);


    }

    @Override
    public void loop() {

        feedBack();
        drive();
        particleLaunch();
        particleCollector();
        servos();
        extender();

    }

    private void feedBack() {

        // Feedback Data
        telemetry.addData("Feedback Data for", "TankOp");
        telemetry.addData("Left Wheels", robot.motor1.getPower());
        telemetry.addData("Right Wheels", robot.motor2.getPower());
        telemetry.addData("Left Launching Power", (robot.motor3.getPower() * -1));
        telemetry.addData("Right Launching Power", (robot.motor4.getPower() * -1));
        telemetry.addData("Belt Speed", robot.motor5.getPower());
        telemetry.addData("Belt Position", robot.motor5.getCurrentPosition());
        telemetry.addData("Collector Speed", robot.motor6.getPower());
        telemetry.addData("Extend Speed", robot.motor7.getPower());
        telemetry.addData("Extend Position", robot.motor7.getCurrentPosition());
        telemetry.addData("Servos 1 and 2", robot.release1.getPosition());

    }

    private void colorSensor() {

        // Color Sensor Mode Changer
        if (LEDmode == 0) {
            robot.colorS1.enableLed(false);
        } else if (LEDmode == 1) {
            robot.colorS1.enableLed(true);
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
        robot.motor1.setPower(gamepad1.left_stick_y);
        robot.motor2.setPower(gamepad1.right_stick_y);

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
                robot.motor5.setPower(-0.25);
            } else if (beltMode == 2) {
                beltMode = 0;
                robot.motor5.setPower(0.25);
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

    private void  extender() {

        // Extend Code
        if (extendMode == 0 && gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0) {
            robot.motor7.setPower(0);
        } else if (extendMode == 1 && robot.motor7.getCurrentPosition() <= Extender_Max) {
            robot.motor7.setPower(0.5);
        } else if (extendMode == 2 && robot.motor7.getCurrentPosition() >= Extender_Min) {
            robot.motor7.setPower(-0.5);
        } else{
            robot.motor7.setPower(0);
        }

        // Limits The Range of the Motors
        if(robot.motor7.getPower() < 0 && robot.motor7.getCurrentPosition() <= Extender_Min){
            robot.motor7.setPower(1);
            extendMode = 0;
        }
        if(robot.motor7.getPower() > 0 && robot.motor7.getCurrentPosition() >= Extender_Max){
            robot.motor7.setPower(-1);
            extendMode = 0;
        }

        // Automatic Extension
        if (gamepad1.x || gamepad1.y) {
            if (extendMode == 0 && gamepad1.x) {
                extendMode = 4;
            } else if (extendMode == 0 && gamepad1.y) {
                extendMode = 3;
            } else if (extendMode == 1 || extendMode == 2) {
                extendMode = 5;
            }
        }
        if (!gamepad1.x && !gamepad1.y) {
            if (extendMode == 3) {
                extendMode = 1;
            } else if (extendMode == 4) {
                extendMode = 2;
            } else if (extendMode == 5) {
                extendMode = 0;
            }
        }

        // Manual Extension
        if(robot.motor7.getCurrentPosition() <= Extender_Max && gamepad1.left_trigger > 0){
            extendMode = 0;
            robot.motor7.setPower(-1);
        }

        if(robot.motor7.getCurrentPosition() >= Extender_Min && gamepad1.right_trigger > 0){
            extendMode = 0;
            robot.motor7.setPower(1);
        }

    }


    @Override
    public void stop() {

    }
}




