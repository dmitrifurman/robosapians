package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Template: AutoOpTest", group="Linear Opmode")
public class AutoOpTest extends LinearOpMode {
    HardwareRobot robot = new HardwareRobot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        DcMotor motor_front_left = robot.motor1;
        DcMotor motor_front_right = robot.motor2;

        waitForStart();
        //Drive forward at half power for 1/2 a second
        motor_front_left.setPower(0.5);
        motor_front_right.setPower(0.5);

        sleep(1000);

        //Stop motors for 1 second
        motor_front_left.setPower(0);
        motor_front_right.setPower(0);

        sleep(1000);

        //Turn right at half power for 1/2 a second
        motor_front_left.setPower(0.5);
        motor_front_right.setPower(-0.5);

        sleep(1000);

        //Stop Motors for 1/2 a second
        motor_front_left.setPower(0);
        motor_front_right.setPower(0);

        sleep(1000);

        //Drive forward at half power for 1/2 a second
        motor_front_left.setPower(0.5);
        motor_front_right.setPower(0.5);

        sleep(1000);

        //Stop Motors for rest of program
        motor_front_left.setPower(0);
        motor_front_right.setPower(0);

        sleep(1000);
    }
}
