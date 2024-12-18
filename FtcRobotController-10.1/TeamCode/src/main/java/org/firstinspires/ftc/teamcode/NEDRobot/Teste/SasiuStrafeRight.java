package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class SasiuStrafeRight extends LinearOpMode {

    DcMotorEx FS,FD,SS,SD;

    @Override
    public void runOpMode() throws InterruptedException {
        FS = hardwareMap.get(DcMotorEx.class,"FS");
        FD = hardwareMap.get(DcMotorEx.class,"FD");
        SS = hardwareMap.get(DcMotorEx.class,"SS");
        SD = hardwareMap.get(DcMotorEx.class,"SD");

        FS.setDirection(DcMotorSimple.Direction.REVERSE);
        SS.setDirection(DcMotorSimple.Direction.REVERSE);

        FS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while(opModeIsActive()){
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;
            double strafe = gamepad1.right_stick_x;

            double FSPower = drive + turn - strafe;
            double FDPower = drive - turn + strafe;
            double SSPower = drive + turn + strafe;
            double SDPower = drive - turn - strafe;
            FS.setPower(FSPower);
            FD.setPower(FDPower);
            SS.setPower(SSPower);
            SD.setPower(SDPower);

        }
    }
}
