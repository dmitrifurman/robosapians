package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Matthew on 10/14/2016.
 */

    @TeleOp(name="Template: Autonomous", group="Linear Opmode")
    public class Autonomous extends LinearOpMode {
        HardwareRobot robot = new HardwareRobot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        DcMotor motor_left = robot.motor_left;
        DcMotor motor_right = robot.motor_right;

        waitForStart();

        motor_left.setPower(1);
        motor_right.setPower(1);

        sleep(3000);
        sleep(1000);



    }
}
