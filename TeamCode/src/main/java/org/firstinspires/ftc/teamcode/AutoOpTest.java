package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Matthew on 10/7/2016.
 */

public class AutoOpTest extends LinearOpMode {
    DcMotor motor_front_left;
    DcMotor motor_front_right;

    @Override
    public void runOpMode() throws InterruptedException {
        motor_front_left = hardwareMap.dcMotor.get("motor_left");
        motor_front_right = hardwareMap.dcMotor.get("motor_right");
        motor_front_right.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        //Drive forward at half power for 1/2 a second
        motor_front_left.setPower(0.5);
        motor_front_right.setPower(0.5);

        sleep(5000);

        //Stop motors for 1 second
        motor_front_left.setPower(0);
        motor_front_right.setPower(0);

        sleep(10000);

        //Turn right at half power for 1/2 a second
        motor_front_left.setPower(0.5);
        motor_front_right.setPower(-0.5);

        sleep(5000);

        //Stop Motors for 1/2 a second
        motor_front_left.setPower(0);
        motor_front_right.setPower(0);

        sleep(5000);

        //Drive forward at half power for 1/2 a second
        motor_front_left.setPower(0.5);
        motor_front_right.setPower(0.5);

        sleep(5000);

        //Stop Motors for rest of program
        motor_front_left.setPower(0);
        motor_front_right.setPower(0);

        sleep(5000);



    }
}
