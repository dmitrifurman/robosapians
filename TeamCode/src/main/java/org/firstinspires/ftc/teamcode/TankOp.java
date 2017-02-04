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
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Tank Op Simple", group = "Linear Opmode")
public class TankOp extends OpMode {

    public enum DriveMode {
        NORMAL_SPEED, SLOW_SPEED, SLOW_TO_NORMAL_TRANSITION, NORMAL_TO_SLOW_TRANSITION
    }

    private HardwareRobot robot = new HardwareRobot();
    private final static double BeltInterval = 600;
    private DriveMode driveMode = DriveMode.NORMAL_SPEED;
    private double armPosition = 0;
    private double armChange = 0.025;
    private double beltMode = 0;
    private double collectMode = 0;
    private double extendMode = 0;
    private double Button = 0;
    private double buttonPosition = 0.375;

    public TankOp() {

    }

    @Override
    public void init() {
        robot.init(hardwareMap);

        // Sets Default Drive Mode
        driveMode = DriveMode.NORMAL_SPEED;

        // Servo Default Position
        armPosition = 1;
        robot.releaseLeft.setPosition(armPosition);
        robot.releaseRight.setPosition(1 - armPosition);
        robot.btnPush.setPosition(buttonPosition);

        // Set Belt Mode
        beltMode = 0;

        // Set Collect Mode
        collectMode = 0;

        // Set Extend Mode
        extendMode = 0;

        // Sets Startng Robot Target Positions to Zero
        robot.beltMotor.setTargetPosition(0);
        robot.extendMotor.setTargetPosition(0);

        // Reset belt encoder
        robot.beltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.beltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    @Override
    public void loop() {

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
        telemetry.addData("Servos 1/2 Pos", robot.releaseLeft.getPosition());
        telemetry.addData("Button Pusher", robot.btnPush.getPosition());
    }

    private void drive() {

        // Drive Code
        if (driveMode == DriveMode.NORMAL_SPEED) {
            robot.leftDrive.setPower(gamepad1.left_stick_y);
            robot.rightDrive.setPower(gamepad1.right_stick_y);
        } else if (driveMode == DriveMode.SLOW_SPEED) {
            robot.leftDrive.setPower(gamepad1.left_stick_y * 0.32);
            robot.rightDrive.setPower(gamepad1.right_stick_y * 0.32);
        }

        if (gamepad1.start) {
            if (driveMode == DriveMode.NORMAL_SPEED) {
                driveMode = DriveMode.NORMAL_TO_SLOW_TRANSITION;
            } else if (driveMode == DriveMode.SLOW_SPEED) {
                driveMode = DriveMode.SLOW_TO_NORMAL_TRANSITION;
            }
        }

        if (!gamepad1.start) {
            if (driveMode == DriveMode.NORMAL_TO_SLOW_TRANSITION) {
                driveMode = DriveMode.SLOW_SPEED;
            } else if (driveMode == DriveMode.SLOW_TO_NORMAL_TRANSITION) {
                driveMode = DriveMode.NORMAL_SPEED;
            }
        }

    }

    private void particleLaunch() {

        // Launching Code
        if (extendMode == 0) {
            robot.leftLaunch.setPower(0.3);
            robot.rightLaunch.setPower(0.3);
        } else {
            robot.leftLaunch.setPower(0);
            robot.rightLaunch.setPower(0);
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
                robot.beltMotor.setPower(-1);
            } else if (beltMode == 2) {
                beltMode = 0;
                robot.beltMotor.setPower(1);
            }
        }

        if (robot.beltMotor.getCurrentPosition() < BeltInterval * -1 || robot.beltMotor.getCurrentPosition() > BeltInterval) {
            robot.beltMotor.setPower(0);
            robot.beltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.beltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        buttonPosition = Range.clip(buttonPosition, 0, 0.75);

        // Updates Servos
        robot.releaseLeft.setPosition(armPosition);
        robot.releaseRight.setPosition(1 - armPosition);
        robot.btnPush.setPosition(buttonPosition);

    }

    private void particleCollector() {

        // Collect Code
        if (collectMode == 0) {
            robot.collectMotor.setPower(0);
        } else if (collectMode == 1) {
            robot.collectMotor.setPower(1);
        } else if (collectMode == 2) {
            robot.collectMotor.setPower(-1);
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

        // Extend Code
        if (extendMode == 0) {
            robot.extendMotor.setPower(0);
        } else if (extendMode == 1) {
            robot.extendMotor.setPower(1);
            armPosition = 0.25;
        } else if (extendMode == 2) {//&& robot.extendMotor.getCurrentPosition() >= Extender_Min) {
            robot.extendMotor.setPower(-1);
            armPosition = 0.25;
        } else if (gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0) {
            robot.extendMotor.setPower(0);
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
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftLaunch.setPower(0);
        robot.rightLaunch.setPower(0);
        robot.beltMotor.setPower(0);
        robot.collectMotor.setPower(0);
        robot.extendMotor.setPower(0);
    }
}




