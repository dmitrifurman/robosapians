package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;

import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.TeamColor;


@Autonomous(name = "AutoEncoder", group = "Linear OpModes")
public class AutoEncoder extends LinearOpMode {

    private HardwareRobot robot = new HardwareRobot();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_FOOT = (12 * (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * (Math.PI)));

    static final double rtTwo = Math.sqrt(2);

    java.lang.String Left = "LEFT";
    java.lang.String Right = "RIGHT";
    double LoR = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initializing");    //

        robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (Objects.equals(TeamColor, "Blue")) {
            Left = "LEFT";
            Right = "RIGHT";
            LoR = 0;
        } else if (Objects.equals(TeamColor, "Red")) {
            Left = "RIGHT";
            Right = "LEFT";
            LoR = 360;
        }


        sleep(2000);

        robot.gyro.calibrate();

        while (robot.gyro.isCalibrating()) {
            idle();
        }

        telemetry.addData("Status: ", "Ready");
        telemetry.update();

        waitForStart();


        Move(1, 2, 3, 0.5);

        gyroTurn((int)(LoR - 45), "LEFT", 6, 0.5);

        Move(1, rtTwo, 3, 0.5);

        Launch(2);

        Move(1, 5 * rtTwo, 6, 0.5);

        gyroTurn((int)(LoR - 90), "LEFT", 6, 0.5);

        beaconPress();

        gyroTurn((int)(LoR - 180), "LEFT", 8, 0.5);

        Move(1, 4, 6, 0.5);

        gyroTurn((int)(LoR - 270), "RIGHT", 8, 0.5);

        beaconPress();

        Move(1, -4.5, 6, 0.5);


        telemetry.addData("Status: ", "Complete");
    }

    public void Move(double speed, double distance, double time, double pause) throws InterruptedException {
        int target;

        target = (int) (distance * COUNTS_PER_FOOT);

        robot.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motor1.setTargetPosition(target);
        robot.motor2.setTargetPosition(target);

        runtime.reset();
        robot.motor1.setPower(speed);
        robot.motor2.setPower(speed);

        while (opModeIsActive() && (runtime.seconds() < time) && (robot.motor1.isBusy() && robot.motor2.isBusy())) {
            idle();
        }

        robot.motor1.setPower(0);
        robot.motor2.setPower(0);

        robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        sleep((int) (1000 * pause));
    }


    private void Launch(double balls) throws InterruptedException {

        runtime.reset();

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


    public void gyroTurn(int angle, java.lang.String Direction, double time, double pause) throws InterruptedException {

        int gyroAngle = angle;

        robot.motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (Objects.equals(Direction, "LEFT")) {
            gyroAngle = 360 - angle;
        }

        runtime.reset();

        if (Objects.equals(Direction, "RIGHT")) {
            robot.motor1.setPower(-0.25);
            robot.motor2.setPower(0.25);
        } else if (Objects.equals(Direction, "LEFT")) {
            robot.motor1.setPower(0.25);
            robot.motor2.setPower(-0.25);
        }

        while (runtime.seconds() < time) {

            telemetry.addData("Gyro: ", robot.gyro.getHeading());
            telemetry.update();

            if ((robot.gyro.getHeading() <= (gyroAngle + 2) && robot.gyro.getHeading() >= (gyroAngle - 2))) {
                break;
            }

        }

        robot.motor1.setPower(0);
        robot.motor2.setPower(0);

        sleep((int) (pause * 1000));
    }


    public void beaconPress() throws InterruptedException {

        sensorTest();

        Move(0.5, 0.3, 3, 0.5);

        Move(0.5, -0.3, 3, 0.5);

        Move(0.5, 0.3, 3, 0.5);

        Move(0.5, -0.3, 3, 0.5);

    }

}
