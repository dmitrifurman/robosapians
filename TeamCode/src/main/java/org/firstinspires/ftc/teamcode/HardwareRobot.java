package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Matthew on 10/7/2016.
 */

public class HardwareRobot {
    public DcMotor motor_front_left = null;
    public DcMotor motor_front_right = null;
    private HardwareMap hwMap = null;

    public HardwareRobot() {

    }

        public void init(HardwareMap hwMap) {
            // save reference to HW Map
            this.hwMap = hwMap;

            // Define and Initialize Motors
            motor_front_left   = hwMap.dcMotor.get("motor_front_left");
            motor_front_right  = hwMap.dcMotor.get("motor_front_right");
            motor_front_right.setDirection(DcMotor.Direction.REVERSE);

            // Set all motors to zero power
            motor_front_left.setPower(0);
            motor_front_right.setPower(0);
    }



}
