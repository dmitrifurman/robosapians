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

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareRobot {
    public DcMotor motor1 = null,
                   motor2 = null,
                   motor3 = null,
                   motor4 = null,
                   motor5 = null,
                   motor6 = null,
                   motor7 = null;
    public Servo release1 = null,
                 release2 = null;
    public ColorSensor colorS1 = null;
    public GyroSensor gyro = null;
    private HardwareMap hwMap = null;

    public HardwareRobot() {

    }

        public void init(HardwareMap hwMap) {
            // save reference to HW Map
            this.hwMap = hwMap;

            // Assigns Names to Motors
            motor1 = hwMap.dcMotor.get("motor_left");
            motor2 = hwMap.dcMotor.get("motor_right");
            motor3 = hwMap.dcMotor.get("launch_left");
            motor4 = hwMap.dcMotor.get("launch_right");
            motor5 = hwMap.dcMotor.get("Belt");
            motor6 = hwMap.dcMotor.get("Collect");
            motor7 = hwMap.dcMotor.get("Winch");

            // Reverse Motors
            motor2.setDirection(DcMotor.Direction.REVERSE);
            motor3.setDirection(DcMotor.Direction.REVERSE);
            motor5.setDirection(DcMotor.Direction.REVERSE);


            // Assigns Names to Servos
            release1 = hwMap.servo.get("servo_arm_right");
            release2 = hwMap.servo.get("servo_arm_left");

            // Assigns Names to Sensors
            colorS1 = hwMap.colorSensor.get("color_sensor");
            gyro = hwMap.gyroSensor.get("gyro_sensor");

    }



}
