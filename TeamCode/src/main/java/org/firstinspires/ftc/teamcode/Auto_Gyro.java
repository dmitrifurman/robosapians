package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Testing", group = "Linear OpModes")
public class Auto_Gyro extends LinearOpMode {

    private HardwareRobot robot = new HardwareRobot();
    private ElapsedTime runtime = new ElapsedTime();
    private Autonomous_Functions Auto = new Autonomous_Functions();

    static final double rtTwo = Math.sqrt(2);

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        Auto.Team_Alliance();

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        //Auto.Encoder_Init();
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Auto.Gyro_Init();
        robot.gyroSensor.calibrate();

        while (robot.gyroSensor.isCalibrating()) {
            idle();
        }

        telemetry.addData("Status: ", "Ready");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status: ", "Running");
        telemetry.update();

        //Move(1, 2, 3, 0.5);

        gyroTurn(90, Autonomous_Functions.Direction.RIGHT, 0.12, 15, 0.5);

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

    public void gyroTurn(int angle, Autonomous_Functions.Direction dir, double speed, double time, double pause) throws InterruptedException {

        int gyroAngleTarget;

        gyroAngleTarget = angle - 8;

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        runtime.reset();

        switch (dir) {

            case LEFT:
                robot.leftDrive.setPower(-speed);
                robot.rightDrive.setPower(speed);
                break;

            case RIGHT:
                gyroAngleTarget = 360-gyroAngleTarget;
                robot.leftDrive.setPower(speed);
                robot.rightDrive.setPower(-speed);
                break;

        }

        while (runtime.seconds() < time) {

            if ((robot.gyroSensor.getHeading() <= (gyroAngleTarget + 2) && robot.gyroSensor.getHeading() >= (gyroAngleTarget - 2))) {
                break;
            }

            telemetry.addData("Current Angle", robot.gyroSensor.getHeading());
            telemetry.addData("Target Angle", angle);
            telemetry.update();

        }

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        sleep((int) (pause * 1000));
    }
}
