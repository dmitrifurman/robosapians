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

package org.firstinspires.ftc.teamcode.TankOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareRobot;


@TeleOp(name = "Tank Op", group = "Linear Opmode")
public class TankOp extends OpMode {

    private enum driveMode {

        NORMAL, SLOW, SLOW_TO_NORMAL, NORMAL_TO_SLOW

    }

    private enum beltMode {

        FORWARD, BACKWARD, STATIC

    }

    private enum collectMode {

        IN, OUT, STATIC, STATIC_TO_IN, STATIC_TO_OUT, MOVING_TO_STATIC

    }

    private enum extendMode {

        UP, DOWN, STATIC, STATIC_TO_UP, STATIC_TO_DOWN, MOVING_TO_STATIC

    }

    private HardwareRobot robot = new HardwareRobot();

    private final static double BeltInterval = 600;

    private driveMode drive = driveMode.NORMAL;
    private beltMode belt = beltMode.STATIC;
    private collectMode collect = collectMode.STATIC;
    private extendMode extend = extendMode.STATIC;

    private double armPosition = 0;
    private double armChange = 0.025;


    public TankOp() {

    }

    @Override
    public void init() {

        robot.init(hardwareMap);

        // Sets intialize Drive Mode
        drive = driveMode.NORMAL;

        // Servo Default Position
        armPosition = 0;

        //Servo set position to initialize value
        //robot.release.setPosition(armPosition);

        // Set Belt Mode
        belt = beltMode.STATIC;

        // Set Collect Mode
        collect = collectMode.STATIC;

        // Set Extend Mode
        extend = extendMode.STATIC;

        // Button Pusher
        robot.btnPushLeft.setPosition(0.3);
        robot.btnPushRight.setPosition(0.7);

        // Reset belt encoder
        robot.beltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.beltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {

        feedback();

        drive();

        particleLaunch();

        particleCollector();

        servos();

        extender();

    }

    private void feedback() {

        // Feedback Data
        telemetry.addData("Feedback Data for", "TankOp");

        telemetry.addData("Left Motor", robot.leftDrive.getPower());
        telemetry.addData("Right Motor", robot.rightDrive.getPower());

        telemetry.addData("Left Launch Power", (robot.leftLaunch.getPower() * -1));
        telemetry.addData("Right Launch Power", (robot.rightLaunch.getPower() * -1));

        telemetry.addData("Belt Speed", robot.beltMotor.getPower());
        telemetry.addData("Belt Pos", robot.beltMotor.getCurrentPosition());

        telemetry.addData("Collector Speed", robot.collectMotor.getPower());

        telemetry.addData("Extend Speed", robot.extendMotor.getPower());
        telemetry.addData("Extend Pos", robot.extendMotor.getCurrentPosition());

        telemetry.addData("Release Pos", robot.release.getPosition());

        telemetry.addData("Button Push Left", robot.btnPushLeft.getPosition());
        telemetry.addData("Button Push Right", robot.btnPushRight.getPosition());

        robot.colorSensor.enableLed(false);
        robot.colorSensor.enableLed(false);

        telemetry.addData("L Hue", robot.colorSensor.argb());
        telemetry.addData("R Hue", robot.colorSensor.argb());

        telemetry.addData("L Red", robot.colorSensor.red());
        telemetry.addData("L Blue", robot.colorSensor.blue());

        telemetry.addData("R Red", robot.colorSensor.red());
        telemetry.addData("R Blue", robot.colorSensor.blue());


    }


    private void drive() {

        // Drive Code
        switch (drive) {

            case NORMAL:
                robot.leftDrive.setPower(gamepad1.left_stick_y);
                robot.rightDrive.setPower(gamepad1.right_stick_y);
                break;

            case SLOW:
                robot.leftDrive.setPower(gamepad1.left_stick_y * 0.7);
                robot.rightDrive.setPower(gamepad1.right_stick_y * 0.7);
                break;

        }

        if (gamepad1.start) {

            switch (drive) {

                case NORMAL:
                    drive = driveMode.NORMAL_TO_SLOW;
                    break;

                case SLOW:
                    drive = driveMode.SLOW_TO_NORMAL;
                    break;

            }

        }

        if (!gamepad1.start) {

            switch (drive) {

                case NORMAL_TO_SLOW:
                    drive = driveMode.SLOW;
                    break;

                case SLOW_TO_NORMAL:
                    drive = driveMode.NORMAL;
                    break;

            }

        }

    }


    private void particleLaunch() {

        // Launching Code
        if (extend == extendMode.STATIC) {

            robot.leftLaunch.setPower(0.55);
            robot.rightLaunch.setPower(0.55);

        } else {

            robot.leftLaunch.setPower(0);
            robot.rightLaunch.setPower(0);

        }


        // Belt Mode Update
        if (belt == beltMode.STATIC) {

            if (gamepad2.a) {

                belt = beltMode.BACKWARD;

            } else if (gamepad2.b) {

                belt = beltMode.FORWARD;

            }

        }

        if (!gamepad2.a && !gamepad2.b) {

            switch (belt) {

                case BACKWARD:
                    robot.beltMotor.setPower(-1);
                    break;

                case FORWARD:
                    robot.beltMotor.setPower(1);
                    break;

            }

        }

        beltReset();

    }


    private void servos() {

        //Servo Position Updater
        if (gamepad1.a) {

            armPosition += armChange;

        }

        if (gamepad1.b) {

            armPosition -= armChange;

        }

        // Updates Servos
        robot.release.setPosition(armPosition);

        // Button Pusher
        robot.btnPushLeft.setPosition(0.8);
        robot.btnPushRight.setPosition(0);

    }


    private void particleCollector() {

        // Collect Code
        switch (collect) {

            case STATIC:
                robot.collectMotor.setPower(0);
                break;

            case IN:
                robot.collectMotor.setPower(1);
                break;

            case OUT:
                robot.collectMotor.setPower(-1);
                break;

        }

        // Collector Mode Updater
        if (gamepad2.x || gamepad2.y) {

            if (collect == collectMode.STATIC && gamepad2.x) {

                collect = collectMode.STATIC_TO_OUT;

            } else if (collect == collectMode.STATIC && gamepad2.y) {

                collect = collectMode.STATIC_TO_IN;

            } else if (collect == collectMode.IN || collect == collectMode.OUT) {

                collect = collectMode.MOVING_TO_STATIC;

            }

        }

        if (!gamepad2.x && !gamepad2.y) {

            switch (collect) {

                case STATIC_TO_IN:
                    collect = collectMode.IN;
                    break;

                case STATIC_TO_OUT:
                    collect = collectMode.OUT;
                    break;

                case MOVING_TO_STATIC:
                    collect = collectMode.STATIC;
                    break;

            }

        }

    }


    private void extender() {

        // Extend Code
        switch (extend) {

            case STATIC:
                robot.extendMotor.setPower(0);
                break;

            case UP:
                robot.extendMotor.setPower(1);
                break;

            case DOWN:
                robot.extendMotor.setPower(-1);
                break;

        }


        // Automatic Extension
        if (gamepad2.left_bumper || gamepad2.right_bumper) {

            if (extend == extendMode.STATIC && gamepad2.left_bumper) {

                extend = extendMode.STATIC_TO_UP;

            } else if (extend == extendMode.STATIC && gamepad2.right_bumper) {

                extend = extendMode.STATIC_TO_DOWN;

            } else if (extend == extendMode.UP || extend == extendMode.DOWN) {

                extend = extendMode.MOVING_TO_STATIC;

            }

        }

        if (!gamepad2.left_bumper && !gamepad2.right_bumper) {

            switch (extend) {

                case STATIC_TO_UP:
                    extend = extendMode.UP;
                    break;

                case STATIC_TO_DOWN:
                    extend = extendMode.DOWN;
                    break;

                case MOVING_TO_STATIC:
                    extend = extendMode.STATIC;
                    break;

            }

        }

    }

    private void beltReset() {

        if (robot.beltMotor.getCurrentPosition() < BeltInterval * -1 || robot.beltMotor.getCurrentPosition() > BeltInterval) {

            belt = beltMode.STATIC;

            robot.beltMotor.setPower(0);

            robot.beltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.beltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }


    @Override
    public void stop() {

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);


        robot.leftLaunch.setPower(0);

        robot.leftLaunch.setPower(0);

        robot.rightLaunch.setPower(0);

        robot.beltMotor.setPower(0);

        robot.collectMotor.setPower(0);

        robot.extendMotor.setPower(0);


    }


}






