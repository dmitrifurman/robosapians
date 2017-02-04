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


@TeleOp(name = "Tank Op Simple", group = "Linear Opmode")
public class TankOp extends OpMode {

    public enum driveMode {

        NORMAL, SLOW, SLOW_TO_NORMAL, NORMAL_TO_SLOW

    }

    public enum beltMode {

        FORWARD, BACKWARD, STATIC

    }

    public enum collectMode {

        IN, OUT, STATIC, STATIC_TO_IN, STATIC_TO_OUT, MOVING_TO_STATIC

    }

    public enum extendMode {

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

        // Sets Default Drive Mode
        drive = driveMode.NORMAL;

        // Servo Default Position
        armPosition = 1;

        robot.releaseLeft.setPosition(armPosition);
        robot.releaseRight.setPosition(1 - armPosition);

        // Set Belt Mode
        belt = beltMode.STATIC;

        // Set Collect Mode
        collect = collectMode.STATIC;

        // Set Extend Mode
        extend = extendMode.STATIC;

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
        if (drive == driveMode.NORMAL) {

            robot.leftDrive.setPower(gamepad1.left_stick_y);
            robot.rightDrive.setPower(gamepad1.right_stick_y);

        } else if (drive == driveMode.SLOW) {

            robot.leftDrive.setPower(gamepad1.left_stick_y * 0.32);
            robot.rightDrive.setPower(gamepad1.right_stick_y * 0.32);

        }

        if (gamepad1.start) {

            if (drive == driveMode.NORMAL) {

                drive = driveMode.NORMAL_TO_SLOW;

            } else if (drive == driveMode.SLOW) {

                drive = driveMode.SLOW_TO_NORMAL;

            }

        }

        if (!gamepad1.start) {

            if (drive == driveMode.NORMAL_TO_SLOW) {

                drive = driveMode.SLOW;

            } else if (drive == driveMode.SLOW_TO_NORMAL) {

                drive = driveMode.NORMAL;

            }

        }

    }


    private void particleLaunch() {

        // Launching Code

        if (extend == extendMode.STATIC) {

            robot.leftLaunch.setPower(0.3);
            robot.rightLaunch.setPower(0.3);

        } else {

            robot.leftLaunch.setPower(0);
            robot.rightLaunch.setPower(0);

        }


        // Belt Mode Update
        if (gamepad2.a) {

            belt = beltMode.BACKWARD;

        } else if (gamepad2.b) {

            belt = beltMode.FORWARD;

        }

        if (!gamepad2.a && !gamepad2.b) {

            if (belt == beltMode.BACKWARD) {

                belt = beltMode.STATIC;
                robot.beltMotor.setPower(-1);

            } else if (belt == beltMode.FORWARD) {

                belt = beltMode.STATIC;
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

        // Updates Servos
        robot.releaseLeft.setPosition(armPosition);
        robot.releaseRight.setPosition(1 - armPosition);

    }


    private void particleCollector() {

        // Collect Code
        if (collect == collectMode.STATIC) {

            robot.collectMotor.setPower(0);

        } else if (collect == collectMode.IN) {

            robot.collectMotor.setPower(1);

        } else if (collect == collectMode.OUT) {

            robot.collectMotor.setPower(-1);

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

            if (collect == collectMode.STATIC_TO_OUT) {

                collect = collectMode.OUT;

            } else if (collect == collectMode.STATIC_TO_IN) {

                collect = collectMode.IN;

            } else if (collect == collectMode.MOVING_TO_STATIC) {

                collect = collectMode.STATIC;

            }

        }

    }


    private void extender() {

        // Extend Code
        if (extend == extendMode.STATIC) {

            robot.extendMotor.setPower(0);

        } else if (extend == extendMode.UP) {

            robot.extendMotor.setPower(1);

        } else if (extend == extendMode.DOWN) {

            robot.extendMotor.setPower(-1);

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

            if (extend == extendMode.STATIC_TO_UP) {

                extend = extendMode.UP;

            } else if (extend == extendMode.STATIC_TO_DOWN) {

                extend = extendMode.DOWN;

            } else if (extend == extendMode.MOVING_TO_STATIC) {

                extend = extendMode.STATIC;

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




