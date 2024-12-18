package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TestExtendoWithoutPID extends LinearOpMode {

    DcMotorEx liftRight,liftLeft;
    public int current_pos;

    @Override
    public void runOpMode() throws InterruptedException {
        liftLeft = hardwareMap.get(DcMotorEx.class,"LeftLiftMotor");
        liftRight = hardwareMap.get(DcMotorEx.class,"RightLiftMotor");

        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()){
            current_pos = liftLeft.getCurrentPosition();

            if(gamepad1.dpad_up){
                liftLeft.setTargetPosition(current_pos+100);
                liftLeft.setPower(1);
                liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftRight.setTargetPosition(current_pos+100);
                liftRight.setPower(1);
                liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad1.dpad_down){
                liftLeft.setTargetPosition(current_pos-100);
                liftLeft.setPower(1);
                liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftRight.setTargetPosition(current_pos-100);
                liftRight.setPower(1);
                liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            telemetry.addData("Positiion",current_pos);
            telemetry.update();
        }
    }
}
