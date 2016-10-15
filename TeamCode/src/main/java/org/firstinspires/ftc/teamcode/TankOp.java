package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Template: TankOp", group="Linear Opmode")
public class TankOp extends OpMode {
    HardwareRobot robot = new HardwareRobot();
    final static double Arm_Min_Range = 0;
    final static double Arm_Max_Range = 1;

    double armPosition = 0;

    double armChange = 0.025;

    double LEDmode = 0;

    // Declares Servos
    //Servo arm1;
    //Servo arm2;

    // Declares Color Sensor
    ColorSensor color1;

    public TankOp() {

    }

    @Override
    public void init() {
        robot.init(hardwareMap);
        // Assigns Names to Servos
        //arm1 = hardwareMap.servo.get("servo_arm_right");
       // arm2 = hardwareMap.servo.get("servo_arm_left");

        // Servo Default Position
       // armPosition = 0;


        // Assigns Names to Sensors
        //color1 = hardwareMap.colorSensor.get("color_sensor");

    }

    @Override
    public void loop() {

        DcMotor motor1 = robot.motor_front_left;
        DcMotor motor2 = robot.motor_front_right;
        // Color Sensor Setup
        //float hsvValues[] = {0,0,0};
        //final float values[] = hsvValues;
        //LEDmode = 0;

        //if(LEDmode == 0){
        //    color1.enableLed(false);
        //} else if(LEDmode == 1){
         //   color1.enableLed(true);
        //}
        //Color.RGBToHSV(color1.red()*8, color1.green()*8, color1.blue()*8, hsvValues);

        // Feedback Data
        telemetry.addData("Feedback Data for", "TankOp");
        telemetry.addData("Left Wheels", motor1.getPower());
        //telemetry.addData("Right Wheels", motor2.getPower());
        //telemetry.addData("Servos 1 and 2", arm1.getPosition());
        //telemetry.addData("Color Sensor: Clear", color1.alpha());
        //telemetry.addData("Color Sensor: Red", color1.red());
        //telemetry.addData("Color Sensor: Green", color1.green());
       // telemetry.addData("Color Sensor: Blue", color1.blue());
       // telemetry.addData("Color Sensor: Hue", hsvValues[0]);


        //Assigns Joystick Values
        float Left_Motors = Range.clip(gamepad1.left_stick_y, -1, 1);
        float Right_Motors = Range.clip(gamepad1.right_stick_y, -1, 1);

        // Tests if a Joystick is Being Used
        if(Left_Motors != 0) {
            motor1.setPower(gamepad1.left_stick_y);
        }
        if(Right_Motors != 0){
            motor2.setPower(gamepad1.right_stick_y);
        }

        // Stops The Motors if Joypads Aren't Used
        if(Left_Motors == 0){
            motor1.setPower(0);
        }
        if(Right_Motors == 0){
            motor2.setPower(0);
        }


        //Servo Position Updater
        //if (gamepad1.a){
          //  armPosition += armChange;
        //}
        //if (gamepad1.b){
          //  armPosition -= armChange;
        //}

        //Limits Servo Movement
        //armPosition = Range.clip(armPosition, Arm_Min_Range, Arm_Max_Range);

        // Updates Servoes
        //arm1.setPosition(armPosition);
        //arm2.setPosition(armPosition);


        // LED Mode Updater
        //if(gamepad1.x) {
          //  if(LEDmode == 0){
            //    LEDmode = 1;
            //} else if(LEDmode == 1){
              //  LEDmode = 0;
            //}
        //}

    }


    @Override
    public void stop() {
    }

}




