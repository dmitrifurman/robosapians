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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareRobot {
    public DcMotor leftDrive = null,
            rightDrive = null;
            //leftLaunch = null,
            //rightLaunch = null,
            //beltMotor = null,
            //collectMotor = null,
            //extendMotor = null;
   // public Servo releaseLeft = null,
            //releaseRight = null,
                   // btnPush = null;
    //public ColorSensor colorSensor = null;
    public GyroSensor gyroSensor = null;
    private HardwareMap hwMap = null;

    public HardwareRobot() {

    }

        public void init(HardwareMap hwMap) {
            // save reference to HW Map
            this.hwMap = hwMap;

            // Assigns Names to Motors
            leftDrive = hwMap.dcMotor.get("Left Motor");
            rightDrive = hwMap.dcMotor.get("Right Motor");
            /*leftLaunch = hwMap.dcMotor.get("Left Launch");
            rightLaunch = hwMap.dcMotor.get("Right Launch");
            beltMotor = hwMap.dcMotor.get("Belt");
            collectMotor = hwMap.dcMotor.get("Collect");
            extendMotor = hwMap.dcMotor.get("Extend");*/

            // Reverse Motors
            leftDrive.setDirection(DcMotor.Direction.REVERSE);
            /*leftLaunch.setDirection(DcMotor.Direction.REVERSE);
            extendMotor.setDirection(DcMotor.Direction.REVERSE);

            /*leftLaunch.setDirection(DcMotor.Direction.REVERSE);
            collectMotor.setDirection(DcMotor.Direction.REVERSE);*/


            // Assigns Names to Servos
            /*releaseLeft = hwMap.servo.get("Release Left");
            releaseRight = hwMap.servo.get("Release Right");
            btnPush = hwMap.servo.get("Button Push");

            // Assigns Names to Sensors
            colorSensor = hwMap.colorSensor.get("Color Sensor");*/
            gyroSensor = hwMap.gyroSensor.get("Gyro Sensor");

    }



}
