package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MotorTester extends LinearOpMode {

    DcMotorEx FS,FD,SS,SD;

    @Override
    public void runOpMode() throws InterruptedException {
        FS=hardwareMap.get(DcMotorEx.class, "FS");
        FD=hardwareMap.get(DcMotorEx.class, "FD");
        SS=hardwareMap.get(DcMotorEx.class, "SS");
        SD=hardwareMap.get(DcMotorEx.class, "SD");
        SS.setDirection(DcMotorSimple.Direction.REVERSE);
        SD.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a)
                FS.setPower(1);
            else if(gamepad1.b)
                FD.setPower(1);
            else if(gamepad1.x)
                SS.setPower(1);
            else if(gamepad1.y)
                SD.setPower(1);

            telemetry.addData("FS velo",FS.getVelocity());
            telemetry.addData("FD velo",FD.getVelocity());
            telemetry.addData("SS velo",SS.getVelocity());
            telemetry.addData("SD velo",SD.getVelocity());
            telemetry.update();
        }
    }
}
