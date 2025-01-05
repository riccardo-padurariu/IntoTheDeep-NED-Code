package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDServo;

@TeleOp
public class TestExtendoWithoutPID extends LinearOpMode {

    DcMotorEx liftRight,liftLeft;
    public int current_pos;
    NEDServo servo1,servo2;

    @Override
    public void runOpMode() throws InterruptedException {
        liftLeft = hardwareMap.get(DcMotorEx.class,"leftExtendoMotor");
        liftRight = hardwareMap.get(DcMotorEx.class,"rightExtendoMotor");

        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);

        servo1 = new NEDServo(hardwareMap.get(Servo.class,"rightIntake"));
        servo2 = new NEDServo(hardwareMap.get(Servo.class,"leftIntake"));

        servo2.setDirection(Servo.Direction.REVERSE);


        waitForStart();
        while(opModeIsActive()){

            servo1.setPosition(0.341);
            servo2.setPosition(0.341);

            if(gamepad1.left_stick_y > 0){
                liftLeft.setPower(0.5);
                liftRight.setPower(0.5);
            } else if (gamepad1.left_stick_y < 0){
                liftLeft.setPower(-0.5);
                liftRight.setPower(-0.5);
            } else {
                liftLeft.setPower(0);
                liftRight.setPower(0);
            }

            telemetry.addData("Positiion",current_pos);
            telemetry.update();
        }
    }
}
