package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;

import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.TeamColor;

@Autonomous(name = "AutoEncoder: Simple without Gyro", group = "Linear OpModes")
public class Auto_Encoder_Center extends LinearOpMode {

    private HardwareRobot robot = new HardwareRobot();
    private ElapsedTime runtime = new ElapsedTime();
    private Autonomous_Functions Auto = new Autonomous_Functions();

    static final double rtTwo = Math.sqrt(2);

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.beltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.beltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        runtime.reset();

        while (runtime.seconds() <= 10) {
            idle();
        }

        Auto.move(0.5, 2, 3, 0.5);

        Auto.launch(2);

        if (Objects.equals(TeamColor, "Red")) {
            Auto.turn(0.5, 0.4, 6, 0.5);
        } else if (Objects.equals(TeamColor, "Blue")) {
            Auto.turn(0.5, -0.4, 6, 0.5);
        }

        Auto.move(1, 1.5 * rtTwo, 6, 1);

    }

}

