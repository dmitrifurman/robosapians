package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;

import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.TeamColor;


public class Autonomous_Functions extends LinearOpMode {

    private HardwareRobot robot = new HardwareRobot();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_FOOT = (12 * (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * (Math.PI)));

    static final double rtTwo = Math.sqrt(2);
    static final double pi = Math.PI;

    final double DSL = 2 / 3;
    final double DSR = 1 / 2;
    final double DDL = pi * rtTwo;
    final double DDR = ((3 * pi * rtTwo) / 4);

    @Override
    public void runOpMode() throws InterruptedException {
    }

    public void move(double speed, double distance, double time, double pause) throws InterruptedException {
        int target;

        target = (int) (distance * COUNTS_PER_FOOT);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftDrive.setTargetPosition(target);
        robot.rightDrive.setTargetPosition(target);

        runtime.reset();
        robot.leftDrive.setPower(speed);
        robot.rightDrive.setPower(speed);

        while (opModeIsActive() && (runtime.seconds() < time) && (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {
            idle();
        }

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep((int) (1000 * pause));
    }

    /*  public void launch(double balls) throws InterruptedException {

          runtime.reset();

          robot.leftLaunch.setPower(-0.2);
          robot.rightLaunch.setPower(-0.2);

          while (opModeIsActive() && (runtime.seconds() < 3)) {
              idle();
          }

          robot.beltMotor.setPower(0.25);

          while ((robot.beltMotor.getCurrentPosition() < 600 * balls) && (robot.beltMotor.getCurrentPosition() > -600 * balls)) {
              idle();
          }

          robot.beltMotor.setPower(0);

          robot.beltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset belt encoder
          robot.beltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Reset mode to use encoder

          runtime.reset();

          robot.leftLaunch.setPower(0);
          robot.rightLaunch.setPower(0);

      }

    */
    public void turn(double speed, double distance, double time, double pause) throws InterruptedException {

        if (Objects.equals(TeamColor, "Blue")) {
            distance = distance * -1;
        }

        int target;

        target = (int) (distance * COUNTS_PER_FOOT);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftDrive.setTargetPosition(-target);
        robot.rightDrive.setTargetPosition(target);

        runtime.reset();
        robot.leftDrive.setPower(-speed);
        robot.rightDrive.setPower(speed);

        while (opModeIsActive() && (runtime.seconds() < time) && (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {
            idle();
        }

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        sleep((int) (1000 * pause));
    }

    public void gyroTurn(int angle, String direction, double time, double pause) throws InterruptedException {

        int gyroAngle = 0;

        gyroAngle = angle - 5;

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        runtime.reset();

        if (direction.equals("LEFT")) {
            robot.leftDrive.setPower(-0.25);
            robot.rightDrive.setPower(0.25);
        } else if (direction.equals("RIGHT")) {
            robot.leftDrive.setPower(0.25);
            robot.rightDrive.setPower(-0.25);
        }

        while (runtime.seconds() < time) {

            telemetry.addData("Gyro: ", robot.gyroSensor.getHeading());
            telemetry.update();

            if ((robot.gyroSensor.getHeading() <= (gyroAngle + 2) && robot.gyroSensor.getHeading() >= (gyroAngle - 2))) {
                break;
            }

        }

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        sleep((int) (pause * 1000));
    }

    public void drift(double leftSpeed, double rightSpeed,
                      double leftDistance, double rightDistance,
                      double time, double pause)
            throws InterruptedException {

        double leftTarget, rightTarget;

        leftTarget = (leftDistance * COUNTS_PER_FOOT);
        rightTarget = (rightDistance * COUNTS_PER_FOOT);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftDrive.setTargetPosition((int) (leftTarget));
        robot.rightDrive.setTargetPosition((int) (rightTarget));

        runtime.reset();
        robot.leftDrive.setPower(leftSpeed);
        robot.rightDrive.setPower(rightSpeed);

        while (opModeIsActive() && (runtime.seconds() < time) && (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {
            idle();
        }

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep((int) (1000 * pause));
    }
}
    /*public void sensorTest() {

        if (opModeIsActive()) {
            robot.colorSensor.enableLed(false);
            if (robot.colorSensor.red() >= 1 && robot.colorSensor.blue() == 0) {
                if (Objects.equals(TeamColor, "Red")) {
                    robot.btnPush.setPosition(0.75);
                } else if (Objects.equals(TeamColor, "Blue")) {
                    robot.btnPush.setPosition(0);
                }
            } else if (robot.colorSensor.blue() >= 1 && robot.colorSensor.red() == 0) {
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

        move(0.5, 0.3, 3, 0.5);

        move(0.5, -0.3, 3, 0.5);

        move(0.5, 0.3, 3, 0.5);

        move(0.5, -0.9, 3, 0.5);

    }

}
*/