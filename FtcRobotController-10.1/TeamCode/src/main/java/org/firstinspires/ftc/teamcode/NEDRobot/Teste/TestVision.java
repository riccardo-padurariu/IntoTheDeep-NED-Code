package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NEDRobot.Vision.CameraRED;

@TeleOp(name = "TestVision")
@Config
public class TestVision extends LinearOpMode {

    public CameraRED camera;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        camera = new CameraRED(hardwareMap);
        FtcDashboard.getInstance().startCameraStream(camera.backCamera, 30);
        while (!isStopRequested() && !isStarted())
        {
            telemetry.addData("position",camera.Case());
            telemetry.addData("avg1", camera.viziune.getAnalysis());
            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()) ;
    }
}