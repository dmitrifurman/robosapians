package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;

import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.TeamColor;

/*
 RISHI AND MATTHEW WHATEVER YOU DO, DO NOT, I REPEAT, DO NOT, CHANGE THIS CODE!!!!!!!!!
 RISHI AND MATTHEW WHATEVER YOU DO, DO NOT, I REPEAT, DO NOT, CHANGE THIS CODE!!!!!!!!!
 RISHI AND MATTHEW WHATEVER YOU DO, DO NOT, I REPEAT, DO NOT, CHANGE THIS CODE!!!!!!!!!
 RISHI AND MATTHEW WHATEVER YOU DO, DO NOT, I REPEAT, DO NOT, CHANGE THIS CODE!!!!!!!!!
 RISHI AND MATTHEW WHATEVER YOU DO, DO NOT, I REPEAT, DO NOT, CHANGE THIS CODE!!!!!!!!!
 RISHI AND MATTHEW WHATEVER YOU DO, DO NOT, I REPEAT, DO NOT, CHANGE THIS CODE!!!!!!!!!
 RISHI AND MATTHEW WHATEVER YOU DO, DO NOT, I REPEAT, DO NOT, CHANGE THIS CODE!!!!!!!!!
 RISHI AND MATTHEW WHATEVER YOU DO, DO NOT, I REPEAT, DO NOT, CHANGE THIS CODE!!!!!!!!!
 RISHI AND MATTHEW WHATEVER YOU DO, DO NOT, I REPEAT, DO NOT, CHANGE THIS CODE!!!!!!!!!
 RISHI AND MATTHEW WHATEVER YOU DO, DO NOT, I REPEAT, DO NOT, CHANGE THIS CODE!!!!!!!!!
 RISHI AND MATTHEW WHATEVER YOU DO, DO NOT, I REPEAT, DO NOT, CHANGE THIS CODE!!!!!!!!!
 RISHI AND MATTHEW WHATEVER YOU DO, DO NOT, I REPEAT, DO NOT, CHANGE THIS CODE!!!!!!!!!
 RISHI AND MATTHEW WHATEVER YOU DO, DO NOT, I REPEAT, DO NOT, CHANGE THIS CODE!!!!!!!!!
 RISHI AND MATTHEW WHATEVER YOU DO, DO NOT, I REPEAT, DO NOT, CHANGE THIS CODE!!!!!!!!!
 RISHI AND MATTHEW WHATEVER YOU DO, DO NOT, I REPEAT, DO NOT, CHANGE THIS CODE!!!!!!!!!
 */
@Autonomous(name = "AutoEncoder: Without Gyro", group = "Linear OpModes")
public class AutoEncoder2 extends LinearOpMode {

    private HardwareRobot robot = new HardwareRobot();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_FOOT = (12 * (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * (Math.PI)));

    static final double rtTwo = Math.sqrt(2);

    private double armPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armPosition = 0.8;
        robot.release1.setPosition(0.8);
        robot.release2.setPosition(.9 - armPosition);

        telemetry.addData("Status: ", "Ready");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status: ", "Running");
        telemetry.update();

        Move(0.5, 2, 3, 0.5);

        if (Objects.equals(TeamColor, "Blue")){
            Turn(0.5, -0.5, 6, 0.5);
        }else if (Objects.equals(TeamColor, "Red")){
            Turn(0.5, 0.5, 6, 0.5);
        }

        Move(1, 0.75*rtTwo, 6, 0.5);

        Launch(2);

        Move(1, 4.75*rtTwo, 6, 0.5);

        if (Objects.equals(TeamColor, "Blue")){
            Turn(0.5, -0.5, 6, 0.5);
        }else if (Objects.equals(TeamColor, "Red")){
            Turn(0.5, 0.5, 6, 0.5);
        }

        Move(0.5, 0.5, 3, 0.5);

        beaconPress();

        if (Objects.equals(TeamColor, "Blue")){
            Turn(0.5, -1, 6, 0.5);
        }else if (Objects.equals(TeamColor, "Red")){
            Turn(0.5, 1, 6, 0.5);
        }

        Move(1, 4, 3, 0.5);

        if (Objects.equals(TeamColor, "Blue")){
            Turn(0.5, 1, 6, 0.5);
        }else if (Objects.equals(TeamColor, "Red")){
            Turn(0.5, -1, 6, 0.5);
        }

        Move(0.5, 1, 3, 0.5);

        beaconPress();

        Move(0.5, -4.5, 6, 0.5);

        telemetry.addData("Status: ", "Complete");
        telemetry.update();
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
            telemetry.addData("Path1",  "Running to %7d :%7d", target, target);
            telemetry.addData("Path2",  "Running at %7d :%7d",
                    robot.motor1.getCurrentPosition(),
                    robot.motor2.getCurrentPosition());
            telemetry.update();

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

        robot.motor3.setPower(-0.3);
        robot.motor4.setPower(-0.3);

        for (int l = 1; l <= balls; l++) {

            while (opModeIsActive() && (runtime.seconds() < 2)) {
                idle();
            }

            robot.motor5.setPower(0.25);

            while ((robot.motor5.getCurrentPosition() < 600) && (robot.motor5.getCurrentPosition() > -600)) {
                idle();
            }

            robot.motor5.setPower(0);

            robot.motor5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset belt encoder
            robot.motor5.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Reset mode to use encoder

            runtime.reset();

        }

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
