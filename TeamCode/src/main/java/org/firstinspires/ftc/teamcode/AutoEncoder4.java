package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;

import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.TeamColor;

@Autonomous(name = "AutoEncoder: Beacons", group = "Linear OpModes")
public class AutoEncoder4 extends LinearOpMode {

    private HardwareRobot robot = new HardwareRobot();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_FOOT = (12 * (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * (Math.PI)));

    static final double rtTwo = Math.sqrt(2);

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        Move(0.5, 2, 3, 0.5);

        if (Objects.equals(TeamColor, "Red")) {
            Turn(0.5, -0.5, 6, 0.5);
        } else if (Objects.equals(TeamColor, "Blue")) {
            Turn(0.5, 0.5, 6, 0.5);
        }

        Move(1, 1.25 * rtTwo, 6, 1);

        if (Objects.equals(TeamColor, "Red")) {
            Turn(0.5, -0.5, 6, 0.5);
        } else if (Objects.equals(TeamColor, "Blue")) {
            Turn(0.5, 0.5, 6, 0.5);
        }

        Move(1, 4.6, 6, 1);

        beaconPress();

        if (Objects.equals(TeamColor, "Red")) {
            Turn(0.5, 0.9, 6, 0.5);
        } else if (Objects.equals(TeamColor, "Blue")) {
            Turn(0.5, -0.9, 6, 0.5);
        }

        Move(1, 4, 6, 1);

        if (Objects.equals(TeamColor, "Red")) {
            Turn(0.5, -0.9, 6, 0.5);
        } else if (Objects.equals(TeamColor, "Blue")) {
            Turn(0.5, 0.9, 6, 0.5);
        }

        Move(0.25, 0.6, 3, 0);

        beaconPress();

        Turn(1, 100, 30, 0);

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

        robot.motor3.setPower(-0.2);
        robot.motor4.setPower(-0.2);

        while (opModeIsActive() && (runtime.seconds() < 3)) {
            idle();
        }

        robot.motor5.setPower(0.25);

        while ((robot.motor5.getCurrentPosition() < 600 * balls) && (robot.motor5.getCurrentPosition() > -600 * balls)) {
            idle();
        }

        robot.motor5.setPower(0);

        robot.motor5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset belt encoder
        robot.motor5.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Reset mode to use encoder

        runtime.reset();

        robot.motor3.setPower(0);
        robot.motor4.setPower(0);

    }


    public void Turn(double speed, double distance, double time, double pause) throws InterruptedException {

        int target;

        target = (int) (distance * COUNTS_PER_FOOT);

        robot.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motor1.setTargetPosition(-target);
        robot.motor2.setTargetPosition(target);

        runtime.reset();
        robot.motor1.setPower(-speed);
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


    public void sensorTest() {

        if (opModeIsActive()) {
            robot.color.enableLed(false);
            if (robot.color.red() >= 1 && robot.color.blue() == 0) {
                if (Objects.equals(TeamColor, "Red")) {
                    robot.btnPush.setPosition(0.75);
                } else if (Objects.equals(TeamColor, "Blue")) {
                    robot.btnPush.setPosition(0);
                }
            } else if (robot.color.blue() >= 1 && robot.color.red() == 0) {
                if (Objects.equals(TeamColor, "Red")) {
                    robot.btnPush.setPosition(0.75);
                } else if (Objects.equals(TeamColor, "Blue")) {
                    robot.btnPush.setPosition(0);
                }
            } else {
                robot.btnPush.setPosition(0.35);
            }
        }
    }

    public void beaconPress() throws InterruptedException {

        sensorTest();

        Move(0.5, 0.3, 3, 0.5);

        Move(0.5, -0.3, 3, 0.5);

        Move(0.5, 0.3, 3, 0.5);

        Move(0.5, -0.9, 3, 0.5);

    }

}

