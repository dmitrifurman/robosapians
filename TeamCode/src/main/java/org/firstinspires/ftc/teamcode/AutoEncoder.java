package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;

import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.TeamColor;

/*

Rishi and Matthew:
        I believe that I have fixed the code for the gyro sensor. Please try it
    and let me know if it works. Other than that, try to get a fully functional
    autonomous. It won't include the launcher or the color sensor because Ethan
    has disassembled the robot. You guys also might want to practice driving for
    the competition next Saturday and the week afterwards. TankOp should be fine,
    so just worry about autonomous for know. If you guys need any pictures of the
    "plan" for autonomous let me know (by text).
                                                                         Thank you,
                                                                             Thomas

*/

@Autonomous(name = "AutoEncoder", group = "Linear OpModes")
public class AutoEncoder extends LinearOpMode {

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * (Math.PI));
    static final double rtTwo = Math.sqrt(2);
    private HardwareRobot robot = new HardwareRobot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initializing");    //
        telemetry.update();

        robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.motor5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.motor5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.gyro.calibrate();

        sleep(2000);

        telemetry.addData("Status: ", "Ready");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status: ", "Running");
        telemetry.update();

        Move(0.5, 18, 3);

        turnGyroTarget(45, 0.5, 6, 'l');

        Move(0.5, rtTwo * 12, 3);

        //Launch(2);

        Move(0.5, 60 * rtTwo, 6);

        turnGyroTarget(45, 0.5, 5, 'l');

        beaconPress();

        turnGyroTarget(90, 0.5, 7, 'l');

        Move(0.5, 48, 6);

        turnGyroTarget(90, 0.5, 7, 'r');

        beaconPress();

        Move(0.5, -54, 6);


        telemetry.addData("Status: ", "Complete");
        telemetry.update();
    }

    public void Move(double speed, double distance, double time) throws InterruptedException {
        int target;

        target = (int) (distance * COUNTS_PER_INCH);

        robot.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motor1.setTargetPosition(target);
        robot.motor2.setTargetPosition(target);

        runtime.reset();
        robot.motor1.setPower(speed);
        robot.motor2.setPower(speed);

        while (opModeIsActive() && (runtime.seconds() < time) && (robot.motor1.isBusy() && robot.motor2.isBusy())) {

            telemetry.addData("Targets", "Running to %7d :%7d", target, target);
            telemetry.addData("Current", "Running at %7d :%7d", robot.motor1.getCurrentPosition(), robot.motor2.getCurrentPosition());
            telemetry.update();

            idle();
        }

        robot.motor1.setPower(0);
        robot.motor2.setPower(0);

        robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    //The below comments are only for the test robot purposes
      /*  private void Launch(double balls)
                throws InterruptedException {

            robot.motor3.setPower(-1);
            robot.motor4.setPower(-1);

            for (int l = 1; l <= balls; l++) {

                while (opModeIsActive() && (runtime.seconds() < 1)) {
                    idle();
                }

                robot.motor5.setPower(0.25);

                while ((robot.motor5.getCurrentPosition() < 600) && (robot.motor5.getCurrentPosition() > -600)) {
                    idle();
                }

                robot.motor5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset belt encoder
                robot.motor5.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Reset mode to use encoder

                runtime.reset();

            }

            robot.motor3.setPower(0);
            robot.motor4.setPower(0);

        }


        public void sensorTest() {

            telemetry.addData("In Sensor Test", "NOW");
            telemetry.update();

            if (opModeIsActive()) {
                robot.color.enableLed(false);
                if (robot.color.red() >= 1 && robot.color.blue() == 0) {
                    if (Objects.equals(TeamColor, "Red")) {
                        robot.btnPush.setPosition(Servo.MAX_POSITION);
                    } else if (Objects.equals(TeamColor, "Blue")) {
                        robot.btnPush.setPosition(Servo.MIN_POSITION);
                    }
                } else if (robot.color.blue() >= 1 && robot.color.red() == 0) {
                    if (Objects.equals(TeamColor, "Red")) {
                        robot.btnPush.setPosition(Servo.MAX_POSITION);
                    } else if (Objects.equals(TeamColor, "Blue")) {
                        robot.btnPush.setPosition(Servo.MIN_POSITION);
                    }
                } else {
                    robot.btnPush.setPosition(Servo.MAX_POSITION / 2);
                }
            }
        }
    */

    public void turnGyroTarget(int angle, double power, int maxTime, char turnDirection) {

        long start = System.currentTimeMillis();
        long end = start + maxTime * 1000; // max time in seconds * 1000 ms per second

        angle -= 16;

        if (turnDirection == 'r') {
            while (robot.gyro.getHeading() < angle && (System.currentTimeMillis() < end)) {
                robot.motor2.setPower(power * -1);
                robot.motor1.setPower(power);
                telemetry.addData("Gyro heading", robot.gyro.getHeading());
                telemetry.update();
            }
        } else if (turnDirection == 'l') {
            do {
                robot.motor1.setPower(power * -1);
                robot.motor2.setPower(power);
                telemetry.addData("Gyro heading", robot.gyro.getHeading());
                telemetry.update();
            }
            while (((robot.gyro.getHeading() > (360 - angle)) || (robot.gyro.getHeading() == 0)) && (System.currentTimeMillis() < end));
        }


        robot.motor2.setPower(0); //turn both motors off
        robot.motor1.setPower(0);

        robot.gyro.calibrate();

        while (robot.gyro.isCalibrating()) {
            //Do nothing
        }
    }

    public void beaconPress() throws InterruptedException {

        /*Move(0.5, 0.6, 3, 0.5);

        //sensorTest();

        Move(0.5, 0.3, 3, 0.5);

        Move(0.5, -0.3, 3, 0.5);

        Move(0.5, 0.3, 3, 0.5);

        Move(0.5, 0.9, 3, 0.5);*/

    }

}