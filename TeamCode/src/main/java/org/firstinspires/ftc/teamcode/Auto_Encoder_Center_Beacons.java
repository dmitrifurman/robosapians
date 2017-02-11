package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;

import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.TeamColor;

@Autonomous(name = "AutoEncoder: Without Gyro", group = "Linear OpModes")
public class Auto_Encoder_Center_Beacons extends LinearOpMode {

    private HardwareRobot robot = new HardwareRobot();
    private ElapsedTime runtime = new ElapsedTime();
    private Autonomous_Functions Auto = new Autonomous_Functions();

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

        Auto.move(0.5, 2, 3, 0.5);

        if (Objects.equals(TeamColor, "Red")){
            Auto.turn(0.5, -0.5, 6, 0.5);
        }else if (Objects.equals(TeamColor, "Blue")){
            Auto.turn(0.5, 0.5, 6, 0.5);
        }

        Auto.move(1, 1.25 * rtTwo, 6, 0.5);

        Auto.move(0.5, -0.6 * rtTwo, 6, 0.5);

        Auto.launch(2);

        Auto.move(1, 4.6 * rtTwo, 6, 0.5);

        if (Objects.equals(TeamColor, "Red")){
            Auto.turn(0.5, -0.5, 6, 0.5);
        }else if (Objects.equals(TeamColor, "Blue")){
            Auto.turn(0.5, 0.5, 6, 0.5);
        }

        Auto.move(0.5, 0.5, 3, 0.5);

        Auto.beaconPress();

        if (Objects.equals(TeamColor, "Red")){
            Auto.turn(0.5, -0.9, 6, 0.5);
        }else if (Objects.equals(TeamColor, "Blue")){
            Auto.turn(0.5, 0.9, 6, 0.5);
        }

        Auto.move(1, 4, 3, 0.5);

        if (Objects.equals(TeamColor, "Red")){
            Auto.turn(0.5, 0.9, 6, 0.5);
        }else if (Objects.equals(TeamColor, "Blue")){
            Auto.turn(0.5, -0.9, 6, 0.5);
        }

        Auto.move(0.5, 1, 3, 0.5);

        Auto.beaconPress();

        Auto.move(0.5, -4.5, 6, 0.5);

    }

}
