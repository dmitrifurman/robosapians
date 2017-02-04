package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;

import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.TeamColor;


@Autonomous(name = "Testing", group = "Linear OpModes")
public class Auto_Gyro extends LinearOpMode {

    private HardwareRobot robot = new HardwareRobot();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_FOOT = (12 * (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * (Math.PI)));

    static final double rtTwo = Math.sqrt(2);

    //java.lang.String Left = "LEFT";
    //java.lang.String Right = "RIGHT";

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        robot.gyroSensor.calibrate();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.beltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.beltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       /* if (Objects.equals(TeamColor, "Red")) {
            Left = "LEFT";
            Right = "RIGHT";
        } else if (Objects.equals(TeamColor, "Blue")) {
            Left = "RIGHT";
            Right = "LEFT";
        }

        while (robot.gyroSensor.isCalibrating()) {
            idle();
        }

        telemetry.addData("Status: ", "Ready");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status: ", "Running");
        telemetry.update();

        //Move(1, 2, 3, 0.5);

        gyroTurn(45, "LEFT", 6, 0.5);

        //Move(1, rtTwo, 3, 0.5);

        //Launch(2);

        /*Move(1, 5 * rtTwo, 6, 0.5);

        gyroTurn(90, Left, 6, 0.5);

        beaconPress();

        gyroTurn(180, Left, 8, 0.5);

        Move(1, 4, 6, 0.5);

        gyroTurn(270, Right, 8, 0.5);

        beaconPress();

        Move(1, -4.5, 6, 0.5);
*/

        telemetry.addData("Status: ", "Complete");
        telemetry.update();
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


   /* private void Launch(double balls) throws InterruptedException {

        runtime.reset();

        robot.leftLaunch.setPower(-0.4);
        robot.rightLaunch.setPower(-0.4);

        for (int l = 1; l <= balls; l++) {

            while (opModeIsActive() && (runtime.seconds() < 1)) {
                idle();
            }

            robot.beltMotor.setPower(0.25);

            while ((robot.beltMotor.getCurrentPosition() < 600) && (robot.beltMotor.getCurrentPosition() > -600)) {
                idle();
            }

            robot.beltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset belt encoder
            robot.beltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Reset mode to use encoder

            runtime.reset();

        }

        robot.leftLaunch.setPower(0);
        robot.rightLaunch.setPower(0);

    }


    public void sensorTest() {

        telemetry.addData("In Sensor Test", "NOW");

        if (opModeIsActive()) {
            robot.colorSensor.enableLed(false);
            if (robot.colorSensor.red() >= 1 && robot.colorSensor.blue() == 0) {
                if (Objects.equals(TeamColor, "Red")) {
                    robot.btnPush.setPosition(Servo.MAX_POSITION);
                } else if (Objects.equals(TeamColor, "Blue")) {
                    robot.btnPush.setPosition(Servo.MIN_POSITION);
                }
            } else if (robot.colorSensor.blue() >= 1 && robot.colorSensor.red() == 0) {
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
    public void gyroTurn(int angle, String direction, double time, double pause) throws InterruptedException {

        int gyroAngle = 0;




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


    public void beaconPress() throws InterruptedException {

        //sensorTest();

        Move(0.5, 0.3, 3, 0.5);

        Move(0.5, -0.3, 3, 0.5);

        Move(0.5, 0.3, 3, 0.5);

        Move(0.5, -0.3, 3, 0.5);

    }

}
