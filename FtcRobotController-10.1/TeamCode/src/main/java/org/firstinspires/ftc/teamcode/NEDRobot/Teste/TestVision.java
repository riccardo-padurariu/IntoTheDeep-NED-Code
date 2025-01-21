package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDServo;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;
import org.firstinspires.ftc.teamcode.NEDRobot.Vision.CameraRED;

@TeleOp(name = "TestVision")
@Config
public class TestVision extends LinearOpMode {

    public CameraRED camera;
    private final Obot obot = Obot.getInstance();
    NEDServo wrist;
    String answer;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        camera = new CameraRED(hardwareMap);
        FtcDashboard.getInstance().startCameraStream(camera.backCamera, 30);
        obot.init(hardwareMap,telemetry);
        obot.read();
        obot.periodic();
        wrist = new NEDServo(hardwareMap.get(Servo.class,"wrist"));
        while (!isStopRequested() && !isStarted())
        {
//            telemetry.addData("position",camera.Case());
//            telemetry.addData("avg1", camera.viziune.getAnalysis1());
//            telemetry.addData("avg2", camera.viziune.getAnalysis2());
//            telemetry.addData("avg3", camera.viziune.getAnalysis3());
//            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()){

            if(camera.Case() == 1) {
                answer = "case left";
                wrist.setPosition(0.5);
            }
            else if(camera.Case() == 2) {
                answer = "case home";
                wrist.setPosition(0.73);
            }
            else if(camera.Case() == 3) {
                answer = "case right";
                wrist.setPosition(0.9);
            }
            telemetry.addData("position",camera.Case());
            telemetry.addData("case wrist",answer);
            telemetry.addData("avg1", camera.viziune.getAnalysis1());
            telemetry.addData("avg2", camera.viziune.getAnalysis2());
            telemetry.addData("avg3", camera.viziune.getAnalysis3());
            telemetry.update();
        }
    }
}