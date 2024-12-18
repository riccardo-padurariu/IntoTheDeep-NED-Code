package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.NEDRobot.Vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@TeleOp
public class TestApril extends LinearOpMode {
    public SampleMecanumDrive drive;
    private GamepadEx GamepadEx1;
    private FtcDashboard ftcDashboard;



    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;
    public static double kP=0;
    public static double kPh=0;

    AprilTagDetection tagOfInterest = null;
    boolean tagFound;
    boolean toggle=false;
    Orientation rot;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard.getInstance().startCameraStream(camera, 30);

        drive=new SampleMecanumDrive(hardwareMap);
        GamepadEx1 = new GamepadEx(gamepad1);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);


        waitForStart();

        while(!isStopRequested())
        {
            tagFound=false;
            tagOfInterest=null;

            drive.setWeightedDrivePower(
                    new Pose2d(
                    GamepadEx1.getLeftY(),
                    GamepadEx1.getLeftX(),
                    -GamepadEx1.getRightX())
            );

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == 4 || tag.id == 5 || tag.id==6) {
                        rot = Orientation.getOrientation(tagOfInterest.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
            if(tagFound) {
                if (gamepad1.left_bumper && tagOfInterest.pose.z * FEET_PER_METER >= 5) {
                    double error_x = kP * (tagOfInterest.pose.z * FEET_PER_METER - 5);
                    double error_heading = kP * (tagOfInterest.pose.z * FEET_PER_METER - 5);
                    double semn = tagOfInterest.pose.z * FEET_PER_METER - 5;
                    double s = Math.signum(semn);
                    drive.setWeightedDrivePower(new Pose2d(0.5 * s, 0, 0));
                }
            }
            /*if(tagFound && gamepad1.left_bumper && rot.firstAngle>=5)
            {
                double angle=5-rot.firstAngle;


                drive.setWeightedDrivePower(new Pose2d(0,0,0.3*angle));
            }*/



                if(tagFound)
            {

                telemetry.addLine(String.format("\nDetected tag ID=%d", tagOfInterest.id));
                telemetry.addLine(String.format("Translation X: %.2f feet", tagOfInterest.pose.x*FEET_PER_METER));
                telemetry.addLine(String.format("Translation Y: %.2f feet", tagOfInterest.pose.y*FEET_PER_METER));
                telemetry.addLine(String.format("Translation Z: %.2f feet", tagOfInterest.pose.z*FEET_PER_METER));
                telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
                telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
                telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
            }
            telemetry.update();
        }
    }
}
