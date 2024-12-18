package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDServo;

@TeleOp
@Config
public class TestServo extends OpMode {

    NEDServo servo1,servo2,servo3,servo4,servo5,servo6;
    public static double pos=0.12;
    DcMotor motor1,motor2,motor3,motor4;
    public static double motor_power=0.142;

    @Override
    public void init() {
        servo1 = new NEDServo(hardwareMap.get(Servo.class,"servo1"));
        servo2 = new NEDServo(hardwareMap.get(Servo.class,"servo2"));
        servo3 = new NEDServo(hardwareMap.get(Servo.class,"servo3"));
        servo4 = new NEDServo(hardwareMap.get(Servo.class,"servo4"));
        servo5 = new NEDServo(hardwareMap.get(Servo.class,"servo5"));
        servo6 = new NEDServo(hardwareMap.get(Servo.class,"servo6"));

        motor1 = hardwareMap.get(DcMotor.class,"motor1");
        motor2 = hardwareMap.get(DcMotor.class,"motor2");
        motor3 = hardwareMap.get(DcMotor.class,"motor3");
        motor4 = hardwareMap.get(DcMotor.class,"motor4");
    }

    @Override
    public void loop() {
        if(pos != 0.12){
            servo1.setPosition(pos);
            servo2.setPosition(pos);
            servo3.setPosition(pos);
            servo4.setPosition(pos);
            servo5.setPosition(pos);
            servo6.setPosition(pos);
        }
        if(motor_power != 0.142){
            motor1.setPower(motor_power);
            motor2.setPower(motor_power);
            motor3.setPower(motor_power);
            motor4.setPower(motor_power);
        }
    }
}
