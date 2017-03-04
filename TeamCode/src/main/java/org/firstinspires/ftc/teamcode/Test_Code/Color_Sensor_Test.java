package org.firstinspires.ftc.teamcode.Test_Code;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

/**
 * Created by Thomas on 3/4/2017.
 */

@Autonomous(name = "Color Concept", group = "Linear OpModes")
public class Color_Sensor_Test extends LinearOpMode {

    byte[] ColorACache,
            ColorCCache;

    DcMotor MLeft,
            MRight;

    I2cDevice ColorA,
            ColorC;

    I2cDeviceSynch ColorAReader,
            ColorCReader;

    int LED = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        MLeft = hardwareMap.dcMotor.get("Left Drive");
        MRight = hardwareMap.dcMotor.get("Right Drive");
        MLeft.setDirection(DcMotor.Direction.REVERSE);

        ColorA = hardwareMap.i2cDevice.get("ColorA");
        ColorC = hardwareMap.i2cDevice.get("ColorC");

        ColorAReader = new I2cDeviceSynchImpl(ColorA, I2cAddr.create8bit(0x3a), false);
        ColorCReader = new I2cDeviceSynchImpl(ColorC, I2cAddr.create8bit(0x3c), false);

        ColorAReader.engage();
        ColorCReader.engage();

        waitForStart();

        while (opModeIsActive()) {

            if (LED == 0) {

                ColorAReader.write8(3, 0);
                ColorCReader.write8(3, 0);

            } else if (LED == 1) {

                ColorAReader.write8(3, 1);
                ColorCReader.write8(3, 1);

            }

            if (gamepad1.a) {
                LED = 1;
            } else {
                LED = 0;
            }

            ColorACache = ColorAReader.read(0x04, 1);
            ColorCCache = ColorCReader.read(0x04, 1);

            telemetry.addData("1 #A", ColorACache[0] & 0xFF);
            telemetry.addData("2 #C", ColorCCache[0] & 0xFF);

            telemetry.addData("3 A", ColorAReader.getI2cAddress().get8Bit());
            telemetry.addData("4 C", ColorCReader.getI2cAddress().get8Bit());

            MLeft.setPower(gamepad1.left_stick_y);
            MRight.setPower(gamepad1.right_stick_y);
        }


    }
}
