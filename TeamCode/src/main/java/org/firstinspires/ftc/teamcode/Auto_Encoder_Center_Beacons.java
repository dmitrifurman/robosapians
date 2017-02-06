package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
/*@Autonomous(name = "AutoEncoder: Without Gyro", group = "Linear OpModes")
public class Auto_Encoder_Center_Beacons extends LinearOpMode {

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

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.beltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.beltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armPosition = 1;
        robot.releaseLeft.setPosition(armPosition);
        robot.releaseRight.setPosition(1 - armPosition);

        runtime.reset();

        while ((robot.leftDrive.getCurrentPosition() != 0 || robot.rightDrive.getCurrentPosition() != 0 || robot.beltMotor.getCurrentPosition() != 0) && runtime.seconds() <= 10) {
            idle();
        }

        waitForStart();

        Move(0.5, 2, 3, 0.5);

        if (Objects.equals(TeamColor, "Red")){
            Turn(0.5, -0.5, 6, 0.5);
        }else if (Objects.equals(TeamColor, "Blue")){
            Turn(0.5, 0.5, 6, 0.5);
        }

        Move(1, 1.25*rtTwo, 6, 0.5);

        Move(0.5, -0.6*rtTwo, 6, 0.5);

        Launch(2);

        Move(1, 4.6*rtTwo, 6, 0.5);

        if (Objects.equals(TeamColor, "Red")){
            Turn(0.5, -0.5, 6, 0.5);
        }else if (Objects.equals(TeamColor, "Blue")){
            Turn(0.5, 0.5, 6, 0.5);
        }

        Move(0.5, 0.5, 3, 0.5);

        beaconPress();

        if (Objects.equals(TeamColor, "Red")){
            Turn(0.5, -0.9, 6, 0.5);
        }else if (Objects.equals(TeamColor, "Blue")){
            Turn(0.5, 0.9, 6, 0.5);
        }

        Move(1, 4, 3, 0.5);

        if (Objects.equals(TeamColor, "Red")){
            Turn(0.5, 0.9, 6, 0.5);
        }else if (Objects.equals(TeamColor, "Blue")){
            Turn(0.5, -0.9, 6, 0.5);
        }

        Move(0.5, 1, 3, 0.5);

        beaconPress();

        Move(0.5, -4.5, 6, 0.5);

    }

    public void Move(double speed, double distance, double time, double pause) throws InterruptedException {
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


    private void Launch(double balls) throws InterruptedException {

        runtime.reset();

        robot.leftLaunch.setPower(-0.3);
        robot.rightLaunch.setPower(-0.3);

        while (opModeIsActive() && (runtime.seconds() < 2)) {
            idle();
        }

        runtime.reset();

        for (int l = 1; l <= balls; l++) {

            while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                idle();
            }

            robot.beltMotor.setPower(0.25);

            while ((robot.beltMotor.getCurrentPosition() < 600) && (robot.beltMotor.getCurrentPosition() > -600)) {
                idle();
            }

            robot.beltMotor.setPower(0);

            robot.beltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset belt encoder
            robot.beltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Reset mode to use encoder

            runtime.reset();

        }

        robot.leftLaunch.setPower(0);
        robot.rightLaunch.setPower(0);

    }


    public void Turn(double speed, double distance, double time, double pause) throws InterruptedException {

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

    public void sensorTest() {

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

        Move(0.5, 0.3, 3, 0.5);

        Move(0.5, -0.3, 3, 0.5);

        Move(0.5, 0.3, 3, 0.5);

        Move(0.5, -0.9, 3, 0.5);

    }

}
*/