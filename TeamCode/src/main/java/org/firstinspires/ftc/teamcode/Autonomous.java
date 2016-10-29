package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

    @TeleOp(name="Template: Autonomous", group="Linear Opmode")
    public class Autonomous extends LinearOpMode {
        HardwareRobot robot = new HardwareRobot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        DcMotor motor_left = robot.motor1;
        DcMotor motor_right = robot.motor2;

        waitForStart();

        robot.motor1.setPower(1);
        robot.motor2.setPower(1);

        sleep(3000);
        sleep(1000);



    }
}
