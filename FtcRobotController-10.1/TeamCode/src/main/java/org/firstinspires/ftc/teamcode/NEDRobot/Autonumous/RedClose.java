package org.firstinspires.ftc.teamcode.NEDRobot.Autonumous;

/*import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDMotorEncoder;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;
import org.firstinspires.ftc.teamcode.NEDRobot.Vision.CameraRED;
import org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NEDRobot.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.ProfileConstraints;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.ProfileState;
import org.firstinspires.ftc.teamcode.photoncore.PhotonCore;

@Autonomous
@Config
public class RedClose extends LinearOpMode {


    public DcMotorEx LeftLiftMotor;
    public DcMotorEx RightLiftMotor;
    public NEDMotorEncoder liftEncoder;
    public double IMU_FREQ=4;
  //  public IMU imu;
    public IMU imu;

    private double positionLift;
    public static double targetposition;
    public double realtarget;
    private double pTargetPosition = 0.0;
    private double pPower =0.0;
    public static double p=0.009,i=0.000,d=0.0000;
    public PIDController controller;
    private ElapsedTime imuTime;
    private AsymmetricMotionProfile profile;
    private ProfileConstraints constraints;
    private ProfileState state;
    public static double velo=4000,accel=4000,decel=4000;
    private double power;
    private final Obot obot = Obot.getInstance();
    private FtcDashboard ftcDashboard;
    public SampleMecanumDrive drive;
    public CameraRED camera;
    public int position;
    public VoltageSensor voltageSensor;
    public Pose2d POSE_START;
    private TrajectorySequence preloadPlaceLeft;
    private TrajectorySequence PickStPixelLeft;
    private TrajectorySequence DropLeft;
    private TrajectorySequence DropLeft2;
    private TrajectorySequence preloadPlaceCenter;
    private TrajectorySequence PickStPixelCenter;
    private TrajectorySequence DropCenter;
    private TrajectorySequence DropCenter2;
    private TrajectorySequence preloadPlaceRight;
    private TrajectorySequence PickStPixelRight;
    private TrajectorySequence DropRight;
    private TrajectorySequence DropRight2;
    private TrajectorySequence ParkLeft;
    private TrajectorySequence ParkCenter;
    private TrajectorySequence ParkRight;
    CommandScheduler commandScheduler1,commandScheduler2,commandScheduler3;
    public static NanoClock clock = NanoClock.system();



    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
        obot.init(hardwareMap,telemetry);
        obot.read();
        obot.Lift.setPID(0.01,0,0);
        // controller = new PIDController(p,i,d);
        //controller.setPID(p,i,d);
        //profile = new AsymmetricMotionProfile(0,0,constraints);
        //timer = new ElapsedTime();
        //state =new ProfileState();

        CommandScheduler.getInstance().reset();
        drive = new SampleMecanumDrive(hardwareMap);
        //SampleMecanumDrive.THREAD_IMU=true;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        commandScheduler1.getInstance().reset();
        commandScheduler2.getInstance().reset();
        commandScheduler3.getInstance().reset();
        imuTime=new ElapsedTime();
        camera = new CameraRED(hardwareMap);
        FtcDashboard.getInstance().startCameraStream(camera.backCamera, 30);
        commandScheduler1.getInstance().reset();
        commandScheduler2.getInstance().reset();
        commandScheduler3.getInstance().reset();

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        PhotonCore.enable();
        telemetry.setMsTransmissionInterval(50);

        ftcDashboard = FtcDashboard.getInstance();

        preloadPlaceRight = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(30,-20,Math.toRadians(-90)))
                .build();
        PickStPixelRight= drive.trajectorySequenceBuilder(preloadPlaceRight.end())
                .lineToLinearHeading(new Pose2d(26,-25,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(19.4,-35.99,Math.toRadians(-90)))
                /*.lineToLinearHeading(new Pose2d(51.9,-18.4,Math.toRadians(90)),
                        getVelocityConstraint(7,Math.toRadians(80),DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(7))
                .build();
        DropRight = drive.trajectorySequenceBuilder(PickStPixelRight.end())
                .lineToLinearHeading(new Pose2d(19.4,-30,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(0.6,-30,Math.toRadians(-90)))
                .build();
        DropRight2 = drive.trajectorySequenceBuilder(DropRight.end())

                .lineToLinearHeading(new Pose2d(29.7,84,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(26,87.2,Math.toRadians(-90)))
                .build();
        ParkRight = drive.trajectorySequenceBuilder(DropRight2.end())
                .lineToLinearHeading(new Pose2d(31,80,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(50,80,Math.toRadians(90)))
                .build();


        /////////////////CENTER//////////////////////////

        preloadPlaceCenter = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(36,-14,Math.toRadians(-90)))
                .build();
        PickStPixelCenter= drive.trajectorySequenceBuilder(preloadPlaceCenter.end())
                .lineToLinearHeading(new Pose2d(36,-20,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(23.8,-35.99,Math.toRadians(-90)))
                .build();

        DropCenter = drive.trajectorySequenceBuilder(PickStPixelCenter.end())
                .lineToLinearHeading(new Pose2d(24.1,-30,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(3,-30,Math.toRadians(-90)))
                .build();

        preloadPlaceLeft = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(25,-0.5,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(26.7,2,Math.toRadians(-90)))
                .build();
        PickStPixelLeft= drive.trajectorySequenceBuilder(preloadPlaceLeft.end())
                .lineToLinearHeading(new Pose2d(31.2,-35.99,Math.toRadians(-90)))
                /*.lineToLinearHeading(new Pose2d(52,-18.4,Math.toRadians(90)),
                        getVelocityConstraint(7,Math.toRadians(80),DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(7))
                .build();
        DropLeft = drive.trajectorySequenceBuilder(PickStPixelLeft.end())
                .lineToLinearHeading(new Pose2d(31.2,-31,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(3,-31,Math.toRadians(-90)))
                .build();




        double loopTime = 0;
        double lastLoopTime = 0;
        double imuAngle = 0.0;



        POSE_START = new Pose2d(0, 0, 0);
        //localizer
        drive.getLocalizer().setPoseEstimate(POSE_START);
        CommandScheduler.getInstance().reset();
        int ok=0;
        while (!isStarted() && !isStopRequested()) {
            if(ok==0)
            {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                               // new ClawPosCommand(obot, IntakeSubsystem.ClawState.CLOSE),
                                new WristPosCommand(obot, IntakeSubsystem.WristState.HOME),
                                new BucketPosCommand(obot, LiftSubsystem.BucketState.HOME),
                                new PitchPosCommand(obot, LiftSubsystem.PitchState.HOME),
                                new WristOuttakePosCommand(obot, LiftSubsystem.WristState.HORIZONTAL_TRANSFER),
                                new TriggerPosCommand(obot, LiftSubsystem.TriggerState.OPEN)
                        ));
                ok=1;
            }
            if(ok<=8)
            {
                CommandScheduler.getInstance().run();
                ok++;
            }

            drive.update();
            position = camera.Case();
            if(imuTime.seconds() > 1.0 / IMU_FREQ) {
                imuTime.reset();
                imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                Pose2d pose = drive.getPoseEstimate();
                drive.setPoseEstimate(new Pose2d(pose.getX(), pose.getY(), imuAngle));
            }


            telemetry.addLine("start");
            telemetry.addData("Position", position);
            telemetry.update();
        }

        new Thread(() -> {
            CameraRED.stop();
        }).start();

        waitForStart();
        if(position==1) {
            commandScheduler1.getInstance().schedule(

                    new SequentialCommandGroup(


                    )
            );
        } else if (position==2) {
            commandScheduler2.getInstance().schedule(

                    new SequentialCommandGroup(

                    )
            );
        }
        else {
            commandScheduler3.getInstance().schedule(

                    new SequentialCommandGroup(

                    )
            );
        }
        while (opModeIsActive() && !isStopRequested()) {

            if (position == 1)
                commandScheduler1.getInstance().run();
            else if (position == 2)
                commandScheduler2.getInstance().run();
            else
                commandScheduler3.getInstance().run();

            obot.clearBulkCache();

            obot.read();
            obot.periodic();
            if(imuTime.seconds() > 1.0 / IMU_FREQ) {
                imuTime.reset();

                imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                Pose2d pose = drive.getPoseEstimate();
                drive.setPoseEstimate(new Pose2d(pose.getX(), pose.getY(), imuAngle));
            }

            obot.write();
            drive.update();


            telemetry.update();




        }
    }
}*/
