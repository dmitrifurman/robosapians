package org.firstinspires.ftc.teamcode.Test_Code;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Thomas on 2/27/2017.
 */

@Autonomous(name = "Gyro Concept", group = "Linear OpModes")
public class New_Auto extends LinearOpMode {

    DcMotor MLeft,
            MRight;

    GyroSensor sensorGyro;
    ModernRoboticsI2cGyro mrGyro;

    private static final double COUNTS_PER_MOTOR_REV = 1120;

    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * (Math.PI));

    @Override
    public void runOpMode() throws InterruptedException {

        MLeft = hardwareMap.dcMotor.get("Left Drive");
        MRight = hardwareMap.dcMotor.get("Right Drive");
        MLeft.setDirection(DcMotor.Direction.REVERSE);

        MLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double turnSpeed = 0.15;

        sensorGyro = hardwareMap.gyroSensor.get("Gyro Sensor");
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;
        int zAccumulated;

        telemetry.addData("Status: ", "Initializing");
        telemetry.update();

        sleep(1000);
        mrGyro.calibrate();

        while (mrGyro.isCalibrating()) {
            idle();
        }

        telemetry.addData("Status: ", "Ready");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status: ", "Running");

        while (opModeIsActive()) {

            turnAbsolute(90);

            sleep(2000);

            turn(90);

            sleep(2000);

            turn(-270);

            sleep(2000);

            turnAbsolute(0);

            sleep(2000);

            driveStraight(12, 0.5);

            sleep(2000);

            driveStraight(-24, 0.5);

            sleep(2000);

            driveStraight(12, 0.5);

            break;

        }

        telemetry.addData("Status: ", "Complete");
        telemetry.update();

    }

    public void driveStraight(int distance, double power) {
        double leftSpeed,
                rightSpeed;

        double target = mrGyro.getIntegratedZValue();
        double startPosition = MLeft.getCurrentPosition();

        double totalDistance = distance * COUNTS_PER_INCH;

        while (MLeft.getCurrentPosition() < totalDistance + startPosition) {
            int zAccumulated = mrGyro.getIntegratedZValue();

            leftSpeed = power + (zAccumulated - target) / 100;
            rightSpeed = power - (zAccumulated - target) / 100;

            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            MLeft.setPower(leftSpeed);
            MRight.setPower(rightSpeed);

            telemetry.addData("Left Power: ", MLeft.getPower());
            telemetry.addData("Right Power: ", MRight.getPower());
            telemetry.addData("Gyro: ", String.format("@03d", zAccumulated));
            telemetry.addData("Distance to go: ", distance + startPosition - MLeft.getCurrentPosition());
            telemetry.update();

        }

        MLeft.setPower(0);
        MRight.setPower(0);

    }

    public void turn(int target) throws InterruptedException {
        turnAbsolute(target + mrGyro.getIntegratedZValue());
    }

    public void turnAbsolute(int target) throws InterruptedException {
        int zAccumulated = mrGyro.getIntegratedZValue();
        double turnSpeed = 0.15;

        while (Math.abs(zAccumulated - target) > 3) {

            if (zAccumulated > target) {
                MLeft.setPower(turnSpeed);
                MRight.setPower(-turnSpeed);
            }

            if (zAccumulated < target) {
                MLeft.setPower(-turnSpeed);
                MRight.setPower(turnSpeed);
            }

            zAccumulated = mrGyro.getIntegratedZValue();
            telemetry.addData("Gyro: ", String.format("@03d", zAccumulated));
            telemetry.update();

        }

        MLeft.setPower(0);
        MRight.setPower(0);

    }


}
