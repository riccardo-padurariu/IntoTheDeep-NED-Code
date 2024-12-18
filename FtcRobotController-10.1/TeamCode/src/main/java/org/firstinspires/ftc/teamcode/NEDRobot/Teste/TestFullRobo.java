package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDServo;

@TeleOp
@Config
public class TestFullRobo extends OpMode {

    NEDServo rightBucket,leftBucket,trigger,wrist,hang;
    public static double bucket_pos=0.012,trigger_pos=0.012,wrist_pos=0.012,hang_pos=0.012;
    NEDServo claw,rightIntake,leftIntake,wrist_intake,pitch;
    public static double claw_pos = 0.012,intake_pos=0.012,wrist_pos_intake=0.012,pitch_pos=0.012;

    @Override
    public void init() {
        rightBucket = new NEDServo(hardwareMap.get(Servo.class,"rightBucket"));
        leftBucket = new NEDServo(hardwareMap.get(Servo.class,"leftBucket"));
        trigger = new NEDServo(hardwareMap.get(Servo.class,"trigger"));
        wrist = new NEDServo(hardwareMap.get(Servo.class,"wrist_outtake"));
        hang = new NEDServo(hardwareMap.get(Servo.class,"hang"));
        leftBucket.setDirection(Servo.Direction.REVERSE);

        claw = new NEDServo(hardwareMap.get(Servo.class,"claw"));
        wrist_intake = new NEDServo(hardwareMap.get(Servo.class,"wrist"));
        pitch = new NEDServo(hardwareMap.get(Servo.class,"pitch_intake"));
        rightIntake = new NEDServo(hardwareMap.get(Servo.class,"rightIntake"));
        leftIntake = new NEDServo(hardwareMap.get(Servo.class,"leftIntake"));
        leftIntake.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if(bucket_pos != 0.012){
            rightBucket.setPosition(bucket_pos/360);
            leftBucket.setPosition(bucket_pos/360);
        }
        if(trigger_pos != 0.012){
            trigger.setPosition(trigger_pos);
        }
        if(wrist_pos != 0.012){
            wrist.setPosition(wrist_pos);
        }
        if(hang_pos != 0.012){
            hang.setPosition(hang_pos);
        }

        if(claw_pos != 0.012)
            claw.setPosition(claw_pos);
        if(wrist_pos_intake != 0.012)
            wrist_intake.setPosition(wrist_pos_intake);
        if(pitch_pos != 0.012)
            pitch.setPosition(pitch_pos);
        if(intake_pos != 0.012) {
            rightIntake.setPosition(intake_pos / 360);
            leftIntake.setPosition(intake_pos / 360);
        }
    }
}
