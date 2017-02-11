package org.firstinspires.ftc.teamcode;

/*import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;

import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.TeamColor;

@Autonomous(name = "AutoEncoder: Beacons", group = "Linear OpModes")
public class Auto_Encoder_Beacons extends LinearOpMode {

    private HardwareRobot robot = new HardwareRobot();
    private ElapsedTime runtime = new ElapsedTime();
    private Autonomous_Functions Auto = new Autonomous_Functions();

    static final double rtTwo = Math.sqrt(2);

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

        while (robot.gyroSensor.isCalibrating()) {
            idle();
        }

        telemetry.addData("Status: ", "Ready");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status: ", "Running");
        telemetry.update();

        Auto.move(0.5, 0.5, 2, 0.5);

        Auto.turn(0.25, -0.5, 3, 0.5);

        Auto.move(0.5, 1.5 * rtTwo, 4, 0.5);

        if (Objects.equals(TeamColor, "Red")) {
            Auto.drift(Auto.DSR, Auto.DSL, Auto.DDR, Auto.DDL, 7, 0.5);
        } else if (Objects.equals(TeamColor, "Blue")) {
            Auto.drift(Auto.DSL, Auto.DSR, Auto.DDL, Auto.DDR, 7, 0.5);
        }


        Auto.move(0.5, -1, 2, 0.5);

        //Sensor Test (new)

        telemetry.addData("Status: ", "Complete");
        telemetry.update();

    }

}
*/
