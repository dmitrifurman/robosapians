package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

    @TeleOp(name="Autonomous", group="Linear Opmode")
    public class Autonomous extends LinearOpMode {
        HardwareRobot robot = new HardwareRobot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        DcMotor motor_left = robot.motor1;
        DcMotor motor_right = robot.motor2;

        motor_left.setTargetPosition(0);
        waitForStart();

/*        motor_left.setPower(1);
        motor_right.setPower(1);

        if(motor_left.getCurrentPosition() == 360){
            motor_left.setPower(0);
            motor_right.setPower(0);
        }
*/
        // Note: 1 wheel rotation = wheel position +/- 1000
        // Note: 1 wheel rotation = 4*pi (12.56) in.

    }

}
