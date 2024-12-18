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
import org.firstinspires.ftc.teamcode.NEDRobot.Vision.CameraREDLeft;
import org.firstinspires.ftc.teamcode.NEDRobot.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NEDRobot.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.ProfileConstraints;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.ProfileState;
import org.firstinspires.ftc.teamcode.photoncore.Neutrino.Rev2MSensor.Rev2mDistanceSensorEx;
import org.firstinspires.ftc.teamcode.photoncore.PhotonCore;

@Autonomous
@Config
public class RedFarBeleaua extends LinearOpMode {


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
    public CameraREDLeft camera;
    public int position;
    public VoltageSensor voltageSensor;
    public Pose2d POSE_START;
    public double IMU_FREQ=4;
    //  public IMU imu;
    public IMU imu;
    public Rev2mDistanceSensorEx distance_sensor;
    private double distance_to_deposit = 13,distance_to_outtaking=40;
    boolean itsTimeToDeposit = false;
    boolean Outtaking = false;
    private TrajectorySequence preloadPlaceLeft;
    private TrajectorySequence PickStPixelLeft;
    private TrajectorySequence DropLeft;
    private TrajectorySequence DropLeft2;
    private TrajectorySequence DropLeft3;
    private TrajectorySequence Pick2Left;
    private TrajectorySequence Pick3Left;
    private TrajectorySequence preloadPlaceCenter;
    private TrajectorySequence PickStPixelCenter;
    private TrajectorySequence DropCenter;
    private TrajectorySequence DropCenter2;
    private TrajectorySequence Pick2Center;
    private TrajectorySequence Pick3Center;
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
    private String[] positions = {
        "",
        "Left",
        "Center",
        "Right",
    };



    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
        imu.resetYaw();
        obot.init(hardwareMap,telemetry);
        obot.read();
        imuTime=new ElapsedTime();
        obot.Lift.setPID(0.01,0,0);
        obot.Intake.periodic();
        distance_sensor = hardwareMap.get(Rev2mDistanceSensorEx.class,"sensorAuto");
        //controller = new PIDController(p,i,d);
        //controller.setPID(p,i,d);
        //profile = new AsymmetricMotionProfile(0,0,constraints);
        //timer = new ElapsedTime();
        //state =new ProfileState();

        CommandScheduler.getInstance().reset();
        drive = new SampleMecanumDrive(hardwareMap);
        //SampleMecanumDrive.THREAD_IMU=true;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        commandScheduler1.getInstance().reset();

        camera = new CameraREDLeft(hardwareMap);
        FtcDashboard.getInstance().startCameraStream(camera.backCamera, 30);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        PhotonCore.enable();
        telemetry.setMsTransmissionInterval(50);

        ftcDashboard = FtcDashboard.getInstance();

        preloadPlaceLeft = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(41,7,Math.toRadians(0)),
                        getVelocityConstraint(120,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(120))
                .build();
        PickStPixelLeft= drive.trajectorySequenceBuilder(preloadPlaceLeft.end())
                .lineToLinearHeading(new Pose2d(45,7,Math.toRadians(0)),
                        getVelocityConstraint(120,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(120))
                .lineToLinearHeading(new Pose2d(52.9,10,Math.toRadians(-90)),
                        getVelocityConstraint(120,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(120))
                .lineToLinearHeading(new Pose2d(52.9,19,Math.toRadians(-90)),
                        getVelocityConstraint(15,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(15))
                .build();
        DropLeft = drive.trajectorySequenceBuilder(PickStPixelLeft.end())
                .lineToLinearHeading(new Pose2d(52.4,-70,Math.toRadians(-90)),//68
                        getVelocityConstraint(80,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(110))
                .splineToSplineHeading(new Pose2d(32.5,-90,Math.toRadians(-90)),Math.toRadians(-180))
                .build();
        Pick2Left = drive.trajectorySequenceBuilder(DropLeft.end())
                .lineToLinearHeading(new Pose2d(54,-75,Math.toRadians(-90)),
                        getVelocityConstraint(80,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(80))
                .lineToLinearHeading(new Pose2d(54,-8,Math.toRadians(-90)),
                        getVelocityConstraint(80,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(80))
                .lineToLinearHeading(new Pose2d(53.9,1.6,Math.toRadians(-90)),
                        getVelocityConstraint(12,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(12))
                .build();
        DropLeft2 = drive.trajectorySequenceBuilder(Pick2Left.end())
                .lineToLinearHeading(new Pose2d(51,-50,Math.toRadians(-90)),
                        getVelocityConstraint(80,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(80))
                .lineToLinearHeading(new Pose2d(26,-83,Math.toRadians(-90)),
                        getVelocityConstraint(80,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(80))
                .lineToLinearHeading(new Pose2d(26,-90,Math.toRadians(-90)),
                        getVelocityConstraint(45,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(45))
                .build();
        Pick3Left = drive.trajectorySequenceBuilder(DropLeft2.end())
                .lineToLinearHeading(new Pose2d(26,-83,Math.toRadians(-90)),
                        getVelocityConstraint(80,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(80))
                .lineToLinearHeading(new Pose2d(55,-75,Math.toRadians(-90)),
                        getVelocityConstraint(80,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(80))
                .lineToLinearHeading(new Pose2d(55,-7,Math.toRadians(-90)),
                        getVelocityConstraint(80,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(80))
                .lineToLinearHeading(new Pose2d(55.2,3,Math.toRadians(-90)),
                        getVelocityConstraint(12,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(12))
                .build();
        DropLeft3 = drive.trajectorySequenceBuilder(Pick3Left.end())
                .lineToLinearHeading(new Pose2d(53.5,3,Math.toRadians(-90)),
                        getVelocityConstraint(80,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(80))
                .lineToLinearHeading(new Pose2d(53.5,-50,Math.toRadians(-90)),
                        getVelocityConstraint(80,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(80))
                .lineToLinearHeading(new Pose2d(27,-83,Math.toRadians(-90)),
                        getVelocityConstraint(80,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(80))
                .lineToLinearHeading(new Pose2d(27,-88,Math.toRadians(-90)),
                        getVelocityConstraint(65,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(65))
                .build();
        ParkLeft = drive.trajectorySequenceBuilder(DropLeft3.end())
                .lineToLinearHeading(new Pose2d(29,-81,Math.toRadians(-90)),
                        getVelocityConstraint(120,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(120))
                .lineToLinearHeading(new Pose2d(48,-81,Math.toRadians(-90)),
                        getVelocityConstraint(120,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(120))
                .build();


        /////////////////CENTER//////////////////////////

        preloadPlaceCenter = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(27,8,Math.toRadians(-180)),
                        getVelocityConstraint(75,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(75))
                .build();

        PickStPixelCenter= drive.trajectorySequenceBuilder(preloadPlaceCenter.end())
                .lineToLinearHeading(new Pose2d(27,14,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(27,21,Math.toRadians(-90)),
                        getVelocityConstraint(15,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(15))
                .build();

        DropCenter = drive.trajectorySequenceBuilder(PickStPixelCenter.end())
                .lineToLinearHeading(new Pose2d(26,7,Math.toRadians(-90)),
                        getVelocityConstraint(75,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(75))
                .lineToLinearHeading(new Pose2d(26,-70,Math.toRadians(-90)),
                        getVelocityConstraint(75,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(75))
                .lineToLinearHeading(new Pose2d(27,-83.5,Math.toRadians(-90)),
                        getVelocityConstraint(75,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(75))
                .build();

        Pick2Center = drive.trajectorySequenceBuilder(DropCenter.end())
                .lineToLinearHeading(new Pose2d(28,-68,Math.toRadians(-90)),
                        getVelocityConstraint(60,DriveConstants.TRACK_WIDTH,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(60))//65
                .lineToLinearHeading(new Pose2d(28,-7,Math.toRadians(-90)),
                        getVelocityConstraint(60,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(60))
                .lineToLinearHeading(new Pose2d(28.5,3,Math.toRadians(-90)),
                        getVelocityConstraint(15,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(15))
                .build();

        DropCenter2 = drive.trajectorySequenceBuilder(Pick2Center.end())
                .lineToLinearHeading(new Pose2d(28,1,Math.toRadians(-90)),
                        getVelocityConstraint(60,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(60))
                .lineToLinearHeading(new Pose2d(28,-82.5,Math.toRadians(-90)),
                        getVelocityConstraint(60,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(60))
                .build();

        Pick3Center = drive.trajectorySequenceBuilder(DropCenter2.end())
                .lineToLinearHeading(new Pose2d(28.5,-15,Math.toRadians(-90)),
                        getVelocityConstraint(60,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(60))
                .lineToLinearHeading(new Pose2d(30,3.5,Math.toRadians(-90)),
                        getVelocityConstraint(15,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(15))
                .build();

        DropCenter3 = drive.trajectorySequenceBuilder(Pick3Center.end())
                .lineToLinearHeading(new Pose2d(28,-50,Math.toRadians(-90)),
                        getVelocityConstraint(60,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(60))
                .lineToLinearHeading(new Pose2d(28,-83,Math.toRadians(-90)),
                        getVelocityConstraint(60,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(60))
                .build();

        ParkCenter = drive.trajectorySequenceBuilder(DropCenter3.end())
                .lineToLinearHeading(new Pose2d(30,-80,Math.toRadians(-90)),
                        getVelocityConstraint(120,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(120))
                .lineToLinearHeading(new Pose2d(48,-80,Math.toRadians(-90)),
                        getVelocityConstraint(120,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(120))
                .build();
        ///////////RIGHT/////////////////////////

        preloadPlaceRight = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(26,-1,Math.toRadians(90)),Math.toRadians(-10))
                .lineToLinearHeading(new Pose2d(30.5,-1.3,Math.toRadians(90)))
                //.lineToLinearHeading(new Pose2d(32,-0.6 ,Math.toRadians(90)))
                .build();
        PickStPixelRight= drive.trajectorySequenceBuilder(preloadPlaceRight.end())
                .lineToLinearHeading(new Pose2d(53.9,14.5,Math.toRadians(-89)))
                .lineToLinearHeading(new Pose2d(53.9,19.9,Math.toRadians(-90)),
                        getVelocityConstraint(7,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(7))
                .build();
        DropRight = drive.trajectorySequenceBuilder(PickStPixelRight.end())
                .lineToLinearHeading(new Pose2d(52,-72,Math.toRadians(-90)))
                .splineToSplineHeading(new Pose2d(20.2,-86.3,Math.toRadians(-90)),Math.toRadians(-180))
              //  .lineToLinearHeading(new Pose2d(19.8,-88,Math.toRadians(-90)),
                //        getVelocityConstraint(30,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                  //      getAccelerationConstraint(25))
                .build();

        Pick2Right = drive.trajectorySequenceBuilder(DropRight.end())
                .lineToLinearHeading(new Pose2d(53,-72,Math.toRadians(-90)))//.91
                .lineToLinearHeading(new Pose2d(53,-7,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(55,17.9,Math.toRadians(-90)))
                .build();
        Pick2RightNow = drive.trajectorySequenceBuilder(Pick2Right.end())
                .lineToLinearHeading(new Pose2d(55,20.15,Math.toRadians(-90)),
                        getVelocityConstraint(7,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(7))
                .build();
        DropRight3 = drive.trajectorySequenceBuilder(Pick2RightNow.end())
                .lineToLinearHeading(new Pose2d(51,-72,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(33,-87.6,Math.toRadians(-90)))
                .build();
        ParkRight = drive.trajectorySequenceBuilder(DropRight3.end())
                .lineToLinearHeading(new Pose2d(33,-80,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(48,-80,Math.toRadians(-90)))
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
                                new ExtendoPosCommand(obot,IntakeSubsystem.ExtendoState.HOME),
                                //new ClawPosCommand(obot, IntakeSubsystem.ClawState.CLOSE),
                                //new WristPosCommand(obot, IntakeSubsystem.WristState.REAL_HOME),
                                //new IntakePosCommand(IntakeSubsystem.IntakeState.REAL_HOME),
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

            telemetry.addLine("Obot is ready");
            telemetry.addData("2+5 running on ", positions[position]);
            telemetry.update();
        }

        new Thread(() -> {
            CameraREDLeft.stop();
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
          /*  else if (position == 2)
                commandScheduler2.getInstance().run();
            else
                commandScheduler3.getInstance().run();

            obot.clearBulkCache();
            /*if(itsTimeToDeposit) {
                double curr_distance = distance_sensor.getDistance(DistanceUnit.CM);
                if (curr_distance <= distance_to_outtaking) {
                    CommandScheduler.getInstance().schedule(
                            new SequentialCommandGroup(
                                    new WaitCommand(500),
                                    new BucketPosCommand(obot, LiftSubsystem.BucketState.LOW),
                                    new PitchPosCommand(obot, LiftSubsystem.PitchState.LOW),
                                    new LiftPosCommand(obot, LiftSubsystem.LiftState.MID)
                            )
                    );
                    Outtaking = true;
                }
                if (curr_distance <= distance_to_deposit && Outtaking) {
                    CommandScheduler.getInstance().schedule(
                            new WaitCommand(200),
                            new TriggerPosCommand(obot, LiftSubsystem.TriggerState.OPEN)
                    );
                    drive.setMotorPowers(0, 0, 0, 0);
                    itsTimeToDeposit = false;
                }
            }

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
            telemetry.addData("hz", loopy);
            telemetry.update();

            loopTime = loop;


        }
    }
}*/
