package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous

        (name="AutonomousEncoderTest", group="Linear Opmode")
public class AutonomousEncoderTest extends LinearOpMode {
    HardwareRobot robot = new HardwareRobot();

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.75;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.motor_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.motor_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.motor_left.getCurrentPosition(),
                robot.motor_right.getCurrentPosition());
        telemetry.update();

        waitForStart();

        encoderDrive(DRIVE_SPEED, 38.0, 38.0, 5.0);
        encoderDrive(TURN_SPEED,   34.0, -34.0, 4.0);
        encoderDrive(DRIVE_SPEED, 50.0, 50.0, 5.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.motor_left.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.motor_right.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.motor_left.setTargetPosition(newLeftTarget);
            robot.motor_right.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.motor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motor_left.setPower(Math.abs(speed));
            robot.motor_right.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motor_left.isBusy() && robot.motor_right.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.motor_left.getCurrentPosition(),
                        robot.motor_right.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.motor_left.setPower(0);
            robot.motor_right.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motor_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motor_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}
