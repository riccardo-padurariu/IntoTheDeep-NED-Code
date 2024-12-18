package org.firstinspires.ftc.teamcode.NEDRobot.Autonumous;

/*import static org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive.getVelocityConstraint;

import com.acmerobotics.dashboard.FtcDashboard;
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
import org.firstinspires.ftc.teamcode.NEDRobot.Vision.CameraRight;
import org.firstinspires.ftc.teamcode.NEDRobot.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NEDRobot.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.ProfileConstraints;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.ProfileState;
import org.firstinspires.ftc.teamcode.photoncore.PhotonCore;

@Autonomous
@Config
public class BlueFarBeleaua extends LinearOpMode {


    public DcMotorEx LeftLiftMotor;
    public DcMotorEx RightLiftMotor;
    public NEDMotorEncoder liftEncoder;
    private double positionLift;
    public static double loopy=0.0;
    public ElapsedTime imuTime;
    public static double targetposition;
    public double realtarget;
    private double pTargetPosition = 0.0;
    private double pPower =0.0;
    public static double p=0.009,i=0.000,d=0.0000;
    public PIDController controller;
    private ElapsedTime timer;
    private AsymmetricMotionProfile profile;
    private ProfileConstraints constraints;
    private ProfileState state;
    public static double velo=4000,accel=4000,decel=4000;
    private double power;

    private final Obot obot = Obot.getInstance();
    private FtcDashboard ftcDashboard;
    public SampleMecanumDrive drive;
    public CameraRight camera;
    public int position;
    public VoltageSensor voltageSensor;
    public Pose2d POSE_START;

    public double IMU_FREQ=4;
    //  public IMU imu;
    public IMU imu;
    private TrajectorySequence preloadPlaceLeft;
    private TrajectorySequence PickStPixelLeft;
    private TrajectorySequence DropLeft;
    private TrajectorySequence DropLeft2;
    private TrajectorySequence DropLeft3;
    private TrajectorySequence Pick2Left;
    private TrajectorySequence Pick2LeftNow;
    private TrajectorySequence preloadPlaceCenter;
    private TrajectorySequence PickStPixelCenter;
    private TrajectorySequence DropCenter;
    private TrajectorySequence DropCenter2;
    private TrajectorySequence Pick2Center;
    private TrajectorySequence Pick2CenterNow;
    private TrajectorySequence DropCenter3;
    private TrajectorySequence preloadPlaceRight;
    private TrajectorySequence PickStPixelRight;
    private TrajectorySequence DropRight;

    private TrajectorySequence DropRight2;
    private TrajectorySequence ParkLeft;
    private TrajectorySequence ParkCenter;
    private TrajectorySequence ParkRight;
    private TrajectorySequence Pick2Right;
    private TrajectorySequence Pick2RightNow;
    private TrajectorySequence DropRight3;
    CommandScheduler commandScheduler1,commandScheduler2,commandScheduler3;
    public static NanoClock clock = NanoClock.system();



    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
        imu.resetYaw();
        obot.init(hardwareMap,telemetry);
        obot.read();
        obot.Lift.setPID(0.01,0,0);
        imuTime=new ElapsedTime();
        // controller = new PIDController(p,i,d);
        //controller.setPID(p,i,d);
        //profile = new AsymmetricMotionProfile(0,0,constraints);
        //timer = new ElapsedTime();
        //state =new ProfileState();

        CommandScheduler.getInstance().reset();
        drive = new SampleMecanumDrive(hardwareMap);
        //SampleMecanumDrive.THREAD_IMU=true;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        camera = new CameraRight(hardwareMap);
        FtcDashboard.getInstance().startCameraStream(camera.backCamera, 30);
        commandScheduler1.getInstance().reset();

        PhotonCore.enable();
        telemetry.setMsTransmissionInterval(50);

        ftcDashboard = FtcDashboard.getInstance();

        preloadPlaceLeft = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(26,1,Math.toRadians(-90)),Math.toRadians(10))
                .lineToLinearHeading(new Pose2d(31.5,1.3,Math.toRadians(-90)))
                .build();
        PickStPixelLeft= drive.trajectorySequenceBuilder(preloadPlaceLeft.end())
                .lineToLinearHeading(new Pose2d(54,-13.8,Math.toRadians(89)))
                .lineToLinearHeading(new Pose2d(54,-19.5,Math.toRadians(90.8)),
                        getVelocityConstraint(7,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(7))
                .build();
        DropLeft = drive.trajectorySequenceBuilder(PickStPixelLeft.end())
                .lineToLinearHeading(new Pose2d(51,72,Math.toRadians(90)),
                        getVelocityConstraint(DriveConstants.MAX_VEL,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(19.8,86.3,Math.toRadians(90)),Math.toRadians(180))

                //  .lineToLinearHeading(new Pose2d(20,86.8,Math.toRadians(90)),
                //        getVelocityConstraint(30,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                  //      getAccelerationConstraint(25))
                .build();


        Pick2Left = drive.trajectorySequenceBuilder(DropLeft.end())
                .lineToLinearHeading(new Pose2d(52,72,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(52,7,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(49.65,-18,Math.toRadians(90)))
                .build();
        Pick2LeftNow = drive.trajectorySequenceBuilder(Pick2Left.end())
                .lineToLinearHeading(new Pose2d(49.65,-20.8,Math.toRadians(90)),
                        getVelocityConstraint(7,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(7))
                .build();
        DropLeft3 = drive.trajectorySequenceBuilder(Pick2LeftNow.end())
                .lineToLinearHeading(new Pose2d(54.3,72,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(25,87.5,Math.toRadians(90)))
                .build();

        ParkLeft = drive.trajectorySequenceBuilder(DropLeft3.end())
                .lineToLinearHeading(new Pose2d(25,80,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(48,80,Math.toRadians(90)))
                .build();


        /////////////////CENTER//////////////////////////

        preloadPlaceCenter = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(51,0,Math.toRadians(0)))
                .build();
        PickStPixelCenter= drive.trajectorySequenceBuilder(preloadPlaceCenter.end())
                .lineToLinearHeading(new Pose2d(53,0,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(54.2,-14,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(54.2,-19.2,Math.toRadians(90)),
                        getVelocityConstraint(7,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(7))
                .build();

        DropCenter = drive.trajectorySequenceBuilder(PickStPixelCenter.end())
                .lineToLinearHeading(new Pose2d(51,72,Math.toRadians(90)))
                .splineToSplineHeading(new Pose2d(24.4,87.4,Math.toRadians(90)),Math.toRadians(180))

                //.lineToLinearHeading(new Pose2d(25.2,87.6,Math.toRadians(90)),
                 //       getVelocityConstraint(30,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                   //     getAccelerationConstraint(25))
                .build();
        Pick2Center = drive.trajectorySequenceBuilder(DropCenter.end())
                .lineToLinearHeading(new Pose2d(52,68,Math.toRadians(90)))//65
                .lineToLinearHeading(new Pose2d(52,12,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(49.4,-17.7,Math.toRadians(90)))
                .build();
        Pick2CenterNow = drive.trajectorySequenceBuilder(Pick2Center.end())
                .lineToLinearHeading(new Pose2d(49.4,-19.9,Math.toRadians(90)),
                        getVelocityConstraint(7,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(7))
                .build();
        DropCenter3 = drive.trajectorySequenceBuilder(Pick2CenterNow.end())
                .lineToLinearHeading(new Pose2d(54,72,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(30,87.4,Math.toRadians(90)))
                .build();
        ParkCenter = drive.trajectorySequenceBuilder(DropCenter3.end())
                .lineToLinearHeading(new Pose2d(31,80,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(48,80,Math.toRadians(90)))
                .build();

        ///////////RIGHT/////////////////////////
        preloadPlaceRight = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(40,-6,Math.toRadians(0)))
                .build();
        PickStPixelRight= drive.trajectorySequenceBuilder(preloadPlaceRight.end())
                .lineToLinearHeading(new Pose2d(47,-6,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(53.8,-13.8,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(53.8,-19.4,Math.toRadians(90)),
                        getVelocityConstraint(7,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(7))
                .build();
        DropRight = drive.trajectorySequenceBuilder(PickStPixelRight.end())
                .lineToLinearHeading(new Pose2d(51.6,72,Math.toRadians(90)))
                .splineToSplineHeading(new Pose2d(30.6,87.5,Math.toRadians(90)),Math.toRadians(180))
                // .lineToLinearHeading(new Pose2d(30.6,88.4,Math.toRadians(90)),
                 //       getVelocityConstraint(30,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                   //     getAccelerationConstraint(25))
                .build();


        Pick2Right = drive.trajectorySequenceBuilder(DropRight.end())
                .lineToLinearHeading(new Pose2d(51.9,72,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(51.9,7,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(49.5,-17.5,Math.toRadians(90)))
                .build();
        Pick2RightNow = drive.trajectorySequenceBuilder(Pick2Right.end())
                .lineToLinearHeading(new Pose2d(49.5,-20,Math.toRadians(90)),
                        getVelocityConstraint(7,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(7))
                .build();
        DropRight3 = drive.trajectorySequenceBuilder(Pick2RightNow.end())
                .lineToLinearHeading(new Pose2d(55,71,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(28,86.99,Math.toRadians(90)))
                .build();
        ParkRight = drive.trajectorySequenceBuilder(DropRight3.end())
                .lineToLinearHeading(new Pose2d(27.8,80,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(48,80,Math.toRadians(90)))
                .build();



        double loopTime = 0;
        double lastLoopTime = 0;


        POSE_START = new Pose2d(0, 0, 0);
        //localizer
        drive.getLocalizer().setPoseEstimate(POSE_START);
        CommandScheduler.getInstance().reset();
        int ok=0;
        double imuAngle = 0.0;
        double imuAngleRad = 0.0;
        while (!isStarted() && !isStopRequested()) {
            if(ok==0)
            {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                //new ClawPosCommand(obot, IntakeSubsystem.ClawState.CLOSE),
                                new WristPosCommand(obot, IntakeSubsystem.WristState.HOME),
                                new BucketPosCommand(obot, LiftSubsystem.BucketState.HOME),
                                new PitchPosCommand(obot, LiftSubsystem.PitchState.HOME),
                                new WristOuttakePosCommand(obot, LiftSubsystem.WristState.HORIZONTAL_TRANSFER),
                                new TriggerPosCommand(obot, LiftSubsystem.TriggerState.OPEN)
                                //new SortPosCommand(LiftSubsystem.SortState.HOME)
                        ));
                ok=1;
            }
            if(ok<=9)
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
            CameraRight.stop();
        }).start();

        waitForStart();
        if(position==1) {
            commandScheduler1.getInstance().schedule(

                    new SequentialCommandGroup(


                    )
            );
        } else if (position==2) {
            commandScheduler1.getInstance().schedule(

                    new SequentialCommandGroup(

                    )
            );
        }
        else {
            commandScheduler1.getInstance().schedule(

                    new SequentialCommandGroup(


                    )

            );
        }
        while (opModeIsActive() && !isStopRequested()) {


           // if (position == 1)
                commandScheduler1.getInstance().run();
            /*else if (position == 2)
                commandScheduler2.getInstance().run();
            else
                commandScheduler3.getInstance().run();

            obot.clearBulkCache();

            obot.read();
            obot.periodic();
            obot.write();
            if(imuTime.seconds() > 1.0 / IMU_FREQ) {
                imuTime.reset();
                imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                Pose2d pose = drive.getPoseEstimate();
                drive.setPoseEstimate(new Pose2d(pose.getX(), pose.getY(), imuAngle));
            }
            drive.update();

            double loop = System.nanoTime();
            loopy=1000000000 / (loop - loopTime);
            telemetry.addData("Pos",obot.Lift.getPosition());
            telemetry.addData("TargetPos",obot.Lift.getTargetPosition());
            telemetry.addData("x", drive.getPoseEstimate().getX());
            telemetry.addData("y", drive.getPoseEstimate().getY());
            telemetry.addData("heading", drive.getPoseEstimate().getHeading());
            telemetry.addData("hz ", loopy);
            telemetry.update();


            loopTime = loop;

        }
    }
}
*/