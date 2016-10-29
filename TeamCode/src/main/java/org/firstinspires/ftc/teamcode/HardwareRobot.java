package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareRobot {
    public DcMotor motor1 = null;
    public DcMotor motor2 = null;
    public DcMotor motor3 = null;
    public DcMotor motor4 = null;
    public DcMotor motor5 = null;
    public DcMotor motor6 = null;
    public DcMotor motor7 = null;
    public Servo arm1 = null;
    public Servo arm2 = null;
    public ColorSensor colorS1 = null;
    private HardwareMap hwMap = null;

    public HardwareRobot() {

    }

        public void init(HardwareMap hwMap) {
            // save reference to HW Map
            this.hwMap = hwMap;

            // Assigns Names to Motors
            motor1 = hwMap.dcMotor.get("motor_left");
            motor2 = hwMap.dcMotor.get("motor_right");
            motor3 = hwMap.dcMotor.get("launch_left");
            motor4 = hwMap.dcMotor.get("launch_right");
            motor5 = hwMap.dcMotor.get("Belt");
            motor6 = hwMap.dcMotor.get("Collect");
            motor7 = hwMap.dcMotor.get("Winch");

            // Reverse Motors
            motor2.setDirection(DcMotor.Direction.REVERSE);
            motor3.setDirection(DcMotor.Direction.REVERSE);
            motor5.setDirection(DcMotor.Direction.REVERSE);


            // Assigns Names to Servos
            arm1 = hwMap.servo.get("servo_arm_right");
            arm2 = hwMap.servo.get("servo_arm_left");

            // Assigns Names to Sensors
            colorS1 = hwMap.colorSensor.get("color_sensor");

    }



}
