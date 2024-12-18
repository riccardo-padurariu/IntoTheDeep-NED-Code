package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDServo;

@TeleOp
@Config
public class TestIntake extends OpMode {

    NEDServo claw,rightIntake,leftIntake,wrist,pitch;
    public static double claw_pos = 0.012,intake_pos=0.012,wrist_pos=0.012,pitch_pos=0.012;
    @Override
    public void init() {
        claw = new NEDServo(hardwareMap.get(Servo.class,"claw"));
        wrist = new NEDServo(hardwareMap.get(Servo.class,"wrist"));
        pitch = new NEDServo(hardwareMap.get(Servo.class,"pitch_intake"));
        rightIntake = new NEDServo(hardwareMap.get(Servo.class,"rightIntake"));
        leftIntake = new NEDServo(hardwareMap.get(Servo.class,"leftIntake"));
        leftIntake.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if(claw_pos != 0.012)
            claw.setPosition(claw_pos);
        if(wrist_pos != 0.012)
            wrist.setPosition(wrist_pos);
        if(pitch_pos != 0.012)
            pitch.setPosition(pitch_pos);
        if(intake_pos != 0.012) {
            rightIntake.setPosition(intake_pos / 360);
            leftIntake.setPosition(intake_pos / 360);
        }
    }
}
