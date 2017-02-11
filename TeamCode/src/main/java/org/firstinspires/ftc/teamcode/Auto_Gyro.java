package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;

import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.TeamColor;


@Autonomous(name = "Testing", group = "Linear OpModes")
public class Auto_Gyro extends LinearOpMode {

    private HardwareRobot robot = new HardwareRobot();
    private ElapsedTime runtime = new ElapsedTime();
    private Autonomous_Functions Auto = new Autonomous_Functions();

    static final double rtTwo = Math.sqrt(2);

    java.lang.String Left = "LEFT";
    java.lang.String Right = "RIGHT";
    double LoR = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        Auto.Team_Alliance();

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        robot.gyroSensor.calibrate();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.beltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.beltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (Objects.equals(TeamColor, "Blue")) {
            Left = "LEFT";
            Right = "RIGHT";
        } else if (Objects.equals(TeamColor, "Red")) {
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

        Auto.gyroTurn(45, Autonomous_Functions.Direction.LEFT, 6, 0.5);

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

}
