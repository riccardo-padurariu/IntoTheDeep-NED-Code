package org.firstinspires.ftc.teamcode.NEDRobot.Teste;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NEDRobot.util.Encoder;

@Config
@TeleOp(name = "TestDrive1")
public class TestDrive1 extends LinearOpMode {

    private SampleMecanumDrive drive1;
    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private Encoder frontEncoder;
    private GamepadEx GamepadEx1;
    private double loopTime = 0.0;


    @Override
    public void runOpMode() throws InterruptedException {
        drive1 = new SampleMecanumDrive(hardwareMap);
        drive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FS"));
        //rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "hang"));
        //frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "SD"));

        GamepadEx1 = new GamepadEx(gamepad1);


        waitForStart();

        while (!isStopRequested()) {

            drive1.setWeightedDrivePower(
                    new Pose2d(dead(scale(GamepadEx1.getLeftY(), 0.6), 0),
                            dead(-scale(GamepadEx1.getLeftX(), 0.6), 0),
                            -GamepadEx1.getRightX()
                    )
            );

            Pose2d poseEstimate = drive1.getPoseEstimate();
            double loop = System.nanoTime();
          /*  telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            loopTime=loop;
            telemetry.update();*/

        }
    }

    public double scale(double x, double k) {
        return (1 - k) * x + k * x * x * x;
    }

    public double dead(double x, double k) {
        return Math.abs(x) > k ? x : 0;
    }

}
