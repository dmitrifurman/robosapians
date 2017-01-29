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

*/

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Tank Op", group = "Linear Opmode")
public class DriverControlv2 extends OpMode {
    private final static double Arm_Min_Range = 0;
    private final static double Arm_Max_Range = 1;
    private final static double Extender_Max = 22000;
    private final static double Extender_Min = 0;
    private final static double BeltInterval = 600;
    private HardwareRobot robot = new HardwareRobot();
    private String driveMode = "normal";
    private double armPosition = 0;
    private double armChange = 0.025;
    private double LEDmode = 0;
    private double beltMode = 0;
    private double collectMode = 0;
    private double extendMode = 0;
    private double power = 0;
    private double Button = 0;
    private double buttonPosition = 0.375;

    public DriverControlv2() {

    }

    @Override
    public void init() {
        robot.init(hardwareMap); // Map all components to corresponding names on Configuration file on phones

        // Set drive mode initialize value
        driveMode = "normal"; //Normal driving: push down joystick fully = 100% motor power

        // Sets button push position
        //robot.btnPush.setPosition(Servo.MAX_POSITION);

        // Servo positioning
        armPosition = Servo.MAX_POSITION; // Set initialized position to maximum position (holding ball raiser lift in stowed position)
        robot.release1.setPosition(armPosition); //Release 1 is set to stowed
        //The direction of release 2 is upside-down so all positions must be opposite of release 1
        robot.release2.setPosition(Servo.MAX_POSITION - armPosition); //Release 2 is set to stowed
        robot.btnPush.setPosition(buttonPosition);

        // Set initialized color sensing mode
        LEDmode = 0;

        // Set Belt Mode
        beltMode = 0;

        // Set Collect Mode
        collectMode = 0;

        // Set Extend Mode
        extendMode = 0;

        power = 0;

        // Reset belt encoder
        robot.motor5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reset extender encoder
        robot.motor7.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor7.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {

        drive();
        particleLaunch();
        particleCollector();
        servos();
        extender();

    }

    /*
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
            telemetry.addData("Extend Speed", robot.motor7.getPower());
            telemetry.addData("Extend Pos", robot.motor7.getCurrentPosition());
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

            if(gamepad1.x){
                robot.btnPush.setPosition(Servo.MAX_POSITION);
            }else if(gamepad1.y){
                robot.btnPush.setPosition(Servo.MIN_POSITION);
            }

        }
    */
    private void drive() {

        // Code to describe driving behavior of two motors
        if (driveMode == "normal") { //normal driving activated
            robot.motor1.setPower(gamepad1.left_stick_y); //If joystick is pushed fully: 100% motor power
            robot.motor2.setPower(gamepad1.right_stick_y); //If joystick is pushed fully: 100% motor power
        } else if (driveMode == "slowed") { //slow mode activated
            robot.motor1.setPower(gamepad1.left_stick_y * 0.32); //If joystick is pushed fully: 32% motor power
            robot.motor2.setPower(gamepad1.right_stick_y * 0.32); //If joystick is pushed fully: 32% motor power
        }

        if (gamepad1.start || gamepad2.start) { //If start key on either gamepad pressed
            if (driveMode == "normal") { //Check if current mode is normal drive
                driveMode = "slowed"; //If normal drive, change to slow mode
            } else { //If not normal drive
                driveMode = "normal"; //Change to normal drive
            }
        }

    }

    private void particleLaunch() {

        //Code to describe launching of particles
        power = Range.clip(power, 0, 0.3);

        // Launching Code
        if (extendMode == 0) {
            robot.motor3.setPower(-0.3 - power);
            robot.motor4.setPower(-0.3 - power);
        } else {
            robot.motor3.setPower(0);
            robot.motor4.setPower(0);
        }


        // Belt Mode Update
        if (gamepad2.a) {
            beltMode = 1;
        } else if (gamepad2.b) {
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

        if (robot.motor5.getCurrentPosition() < BeltInterval * -1 || robot.motor5.getCurrentPosition() > BeltInterval) {
            robot.motor5.setPower(0);
            robot.motor5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motor5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (gamepad2.dpad_up) {
            power -= 0.1;
        } else if (gamepad2.dpad_down) {
            power += 0.1;
        }


    }

    private void servos() {

        //Servo Position Updater
        if (gamepad1.a) {
            armPosition += armChange;
        }
        if (gamepad1.b) {
            armPosition -= armChange;
        }

        if (Button == 0) {

            if (gamepad1.dpad_left) {
                Button = 1;
            } else if (gamepad1.dpad_right) {
                Button = 2;
            }

        }

        if (!gamepad1.dpad_left && !gamepad1.dpad_right) {
            if (Button == 1) {
                Button = 0;
                buttonPosition += 0.375;
            } else if (Button == 2) {
                Button = 0;
                buttonPosition -= 0.375;
            }
        }

        //Limits Servo Movement
        armPosition = Range.clip(armPosition, Arm_Min_Range, Arm_Max_Range);
        buttonPosition = Range.clip(buttonPosition, 0, 0.75);

        // Updates Servos
        robot.release1.setPosition(armPosition);
        robot.release2.setPosition(1 - armPosition);
        robot.btnPush.setPosition(buttonPosition);

    }

    private void particleCollector() {

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

    private void extender() {

        // Limits The Range of the Motors
        /*if(robot.motor7.getPower() < 0 && robot.motor7.getCurrentPosition() < Extender_Min){
            extendMode = 0;
        }
        if(robot.motor7.getPower() > 0 && robot.motor7.getCurrentPosition() > Extender_Max){
            extendMode = 0;
        }
        */

        // Extend Code
        if (extendMode == 0) {
            robot.motor7.setPower(0);
        } else if (extendMode == 1) {//&& robot.motor7.getCurrentPosition() <= Extender_Max) {
            robot.motor7.setPower(1);
            armPosition = 0.25;
        } else if (extendMode == 2) {//&& robot.motor7.getCurrentPosition() >= Extender_Min) {
            robot.motor7.setPower(-1);
            armPosition = 0.25;
        } else if (gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0) {
            robot.motor7.setPower(0);
        }

        // Automatic Extension
        if (gamepad2.left_bumper || gamepad2.right_bumper) {
            if (extendMode == 0 && gamepad2.left_bumper) {
                extendMode = 4;
                armPosition = 0.25;
            } else if (extendMode == 0 && gamepad2.right_bumper) {
                extendMode = 3;
                armPosition = 0.25;
            } else if (extendMode == 1 || extendMode == 2) {
                extendMode = 5;
                armPosition = 0.25;
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


    }


    @Override
    public void stop() {
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
        robot.motor5.setPower(0);
        robot.motor6.setPower(0);
        robot.motor7.setPower(0);
    }
}




