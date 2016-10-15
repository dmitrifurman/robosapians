package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Matthew on 10/7/2016.
 */

public class HardwareRobot {
    public DcMotor motor_left = null;
    public DcMotor motor_right = null;
    private HardwareMap hwMap = null;

    public HardwareRobot() {

    }

        public void init(HardwareMap hwMap) {
            // save reference to HW Map
            this.hwMap = hwMap;

            // Define and Initialize Motors
            motor_left   = hwMap.dcMotor.get("motor_left");
            motor_right  = hwMap.dcMotor.get("motor_right");
            motor_right.setDirection(DcMotor.Direction.REVERSE);

            // Set all motors to zero power
            motor_left.setPower(0);
            motor_right.setPower(0);
    }



}
