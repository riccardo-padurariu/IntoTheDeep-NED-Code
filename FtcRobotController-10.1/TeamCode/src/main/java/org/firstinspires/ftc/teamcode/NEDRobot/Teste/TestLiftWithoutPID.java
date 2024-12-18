package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDMotorEncoder;

@TeleOp
@Config
public class TestLiftWithoutPID extends LinearOpMode {

    public DcMotorEx leftMotor;
    public DcMotorEx rightMotor;
    public NEDMotorEncoder encoder;
    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.get(DcMotorEx.class,"leftLiftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class,"rightLiftMotor");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        encoder = new NEDMotorEncoder(new MotorEx(hardwareMap, "leftLiftMotor").encoder);


        waitForStart();
        while(!isStopRequested() && opModeIsActive())
        {
            leftMotor.setPower(-gamepad1.left_stick_y);
            rightMotor.setPower(-gamepad1.left_stick_y);
            leftMotor.getPower();
            telemetry.addData("LeftMCurr",leftMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("RightMCurr",rightMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("EncoderLeft",leftMotor.getVelocity());
            telemetry.addData("EncoderRight",rightMotor.getVelocity());
            telemetry.addData("Encoder",encoder.getPosition()
            );
            telemetry.update();
        }
    }
}
