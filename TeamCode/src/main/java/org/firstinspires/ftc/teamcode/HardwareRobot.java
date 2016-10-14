package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Matthew on 10/7/2016.
 */

public class HardwareRobot {
    public DcMotor motor_front_left = null;
    public DcMotor motor_front_right = null;

    HardwareMap hwMap = null;

    public HardwareRobot() {

    }

        public void init(HardwareMap ahwMap) {
            // save reference to HW Map
            hwMap = ahwMap;

            // Define and Initialize Motors
            motor_front_left   = hwMap.dcMotor.get("left motor");
            motor_front_right  = hwMap.dcMotor.get("right motor");
            motor_front_right.setDirection(DcMotor.Direction.REVERSE);

            // Set all motors to zero power
            motor_front_left.setPower(0);
            motor_front_right.setPower(0);
    }



}
