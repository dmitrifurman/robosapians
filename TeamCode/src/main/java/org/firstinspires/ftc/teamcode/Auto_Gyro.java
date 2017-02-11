package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

        Auto.Encoder_Init();

        Auto.Gyro_Init();

        telemetry.addData("Status: ", "Ready");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status: ", "Running");
        telemetry.update();

        //Move(1, 2, 3, 0.5);

        Auto.gyroTurn(45, Autonomous_Functions.Direction.LEFT, 0.25, 6, 0.5);

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
