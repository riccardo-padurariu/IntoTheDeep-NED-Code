package org.firstinspires.ftc.teamcode.NEDRobot.Autonumous;

import static org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive.getVelocityConstraint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.IntakeCommands.ClawPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.IntakeCommands.ExtendoPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.IntakeCommands.IntakePosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.IntakeCommands.PitchPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.IntakeCommands.WristPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.OuttakeCommands.BucketPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.OuttakeCommands.LiftPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.OuttakeCommands.TriggerPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDMotorEncoder;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;
import org.firstinspires.ftc.teamcode.NEDRobot.Vision.Camera;
import org.firstinspires.ftc.teamcode.NEDRobot.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NEDRobot.trajectorysequence.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.ProfileConstraints;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.ProfileState;
import org.firstinspires.ftc.teamcode.photoncore.PhotonCore;

@Autonomous
@Config
public class BlueBasketBeleaua extends LinearOpMode {


    public DcMotorEx LeftLiftMotor;
    public DcMotorEx RightLiftMotor;
    public NEDMotorEncoder liftEncoder;
    private double positionLift;
    public ElapsedTime imuTime;

    public static double targetposition;
    public double realtarget;
    private double pTargetPosition = 0.0;
    private double pPower =0.0;
    public static double p=0.006,i=0.000,d=0.0000;
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
    public int position;
    public VoltageSensor voltageSensor;
    public Pose2d POSE_START;
    public double IMU_FREQ=4;
    //  public IMU imu;
    public IMU imu;
    private TrajectorySequence depositPreload,pick1,deposit1,pick2,deposit2,pick3,deposit3;
    CommandScheduler commandScheduler1,commandScheduler2,commandScheduler3;
    public static NanoClock clock = NanoClock.system();



    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        obot.init(hardwareMap,telemetry);
        obot.read();
        obot.Extendo.setPID(p,i,0.000);
        obot.Lift.setPID(0.009,i,0.000);
        // controller = new PIDController(p,i,d);
        //controller.setPID(p,i,d);
        //profile = new AsymmetricMotionProfile(0,0,constraints);
        //timer = new ElapsedTime();
        //state =new ProfileState();

        CommandScheduler.getInstance().reset();
        drive = new SampleMecanumDrive(hardwareMap);
        //SampleMecanumDrive.THREAD_IMU=true;
        imuTime=new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        commandScheduler1.getInstance().reset();
        commandScheduler2.getInstance().reset();
        commandScheduler3.getInstance().reset();
        commandScheduler1.getInstance().reset();
        commandScheduler2.getInstance().reset();
        commandScheduler3.getInstance().reset();

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        PhotonCore.enable();
        telemetry.setMsTransmissionInterval(50);

        ftcDashboard = FtcDashboard.getInstance();




        double loopTime = 0;
        double imuAngle = 0.0;

        depositPreload = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(25,-4,Math.toRadians(45)),
                        getVelocityConstraint(65,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(65))
                .build();

        pick1 = drive.trajectorySequenceBuilder(depositPreload.end())
                .lineToLinearHeading(new Pose2d(18,-11.5,Math.toRadians(90)),
                        getVelocityConstraint(45,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(45))
                .build();

        deposit1 = drive.trajectorySequenceBuilder(pick1.end())
                .lineToLinearHeading(new Pose2d(25,-5,Math.toRadians(45)),
                        getVelocityConstraint(35,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(35))
                .build();

        pick2 = drive.trajectorySequenceBuilder(deposit1.end())
                .lineToLinearHeading(new Pose2d(28.5,-11.5,Math.toRadians(90)),
                        getVelocityConstraint(35,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(35))
                .build();

        deposit2 = drive.trajectorySequenceBuilder(pick2.end())
                .lineToLinearHeading(new Pose2d(25,-3,Math.toRadians(45)),
                        getVelocityConstraint(35,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(35))
                .build();

        pick3 = drive.trajectorySequenceBuilder(deposit2.end())
                .lineToLinearHeading(new Pose2d(14,-36.5,Math.toRadians(180)),
                        getVelocityConstraint(45,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(45))
                .build();

        deposit3 = drive.trajectorySequenceBuilder(pick3.end())
                .lineToLinearHeading(new Pose2d(26.5,-6,Math.toRadians(45)),
                        getVelocityConstraint(45,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(45))
                .build();

        double lastLoopTime = 0;


        POSE_START = new Pose2d(0, 0, 0);
        //localizer
        drive.getLocalizer().setPoseEstimate(POSE_START);
        CommandScheduler.getInstance().reset();
        int ok=0;
        while (!isStarted() && !isStopRequested()) {
            if(ok==0)
            {
                obot.periodic();
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new ExtendoPosCommand(obot, IntakeSubsystem.ExtendoState.HOME),
                                new LiftPosCommand(obot, LiftSubsystem.LiftState.HOME),
                                new IntakePosCommand(obot, IntakeSubsystem.IntakeState.INTAKE),
                                new PitchPosCommand(obot, IntakeSubsystem.PitchState.INTAKE),
                                new ClawPosCommand(obot, IntakeSubsystem.ClawState.OPEN),
                                new WristPosCommand(obot, IntakeSubsystem.WristState.HOME),
                                new BucketPosCommand(obot, LiftSubsystem.BucketState.TRANSFER),
                                new TriggerPosCommand(obot, LiftSubsystem.TriggerState.CLOSE)
                        ));
                ok=1;
            }
            if(ok<=8)
            {
                CommandScheduler.getInstance().run();
                ok++;
            }

            drive.update();
            if(imuTime.seconds() > 1.0 / IMU_FREQ) {
                imuTime.reset();
                imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                Pose2d pose = drive.getPoseEstimate();
                drive.setPoseEstimate(new Pose2d(pose.getX(), pose.getY(), imuAngle));
            }


            telemetry.addLine("Obot is ready");
            telemetry.update();
        }



        waitForStart();
        commandScheduler1.getInstance().schedule(

                new SequentialCommandGroup(
                        ///////////////////PRELOAD/////////////////////////////
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drive,depositPreload),
                                new SequentialCommandGroup(
                                        new LiftPosCommand(obot, LiftSubsystem.LiftState.HIGH_BASKET),
                                        new BucketPosCommand(obot, LiftSubsystem.BucketState.BASKET),
                                        new WaitCommand(1800),
                                        new TriggerPosCommand(obot, LiftSubsystem.TriggerState.OPEN)
                                )
                        ),
                        ///////////////////////PICK1//////////////////////////
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drive,pick1),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new BucketPosCommand(obot, LiftSubsystem.BucketState.TRANSFER),
                                        new WaitCommand(200),
                                        new LiftPosCommand(obot, LiftSubsystem.LiftState.HOME),
                                        new WaitCommand(700),
                                        new InstantCommand(() -> obot.Extendo.setMotionProfileTargetPosition(800))
                                )
                        ),
                        //new WaitCommand(600),
                        //new InstantCommand(() -> obot.Extendo.setMotionProfileTargetPosition(800)),
                        //new WaitCommand(600),
                        new WaitCommand(800),
                        new ClawPosCommand(obot, IntakeSubsystem.ClawState.CLOSE),
                        new WaitCommand(600),
                        //new ExtendoPosCommand(obot, IntakeSubsystem.ExtendoState.HOME),
                        /////////////////////DEPOSIT1////////////////////////////
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drive,deposit1),
                                new SequentialCommandGroup(
                                        new ExtendoPosCommand(obot, IntakeSubsystem.ExtendoState.HOME),
                                        new WristPosCommand(obot, IntakeSubsystem.WristState.HOME),
                                        new WaitCommand(200),
                                        new IntakePosCommand(obot, IntakeSubsystem.IntakeState.TRANSFER),
                                        new WaitCommand(200),
                                        new PitchPosCommand(obot, IntakeSubsystem.PitchState.TRANSFER),
                                        new WaitCommand(600),
                                        new TriggerPosCommand(obot, LiftSubsystem.TriggerState.CLOSE),
                                        new WaitCommand(300),
                                        new ClawPosCommand(obot, IntakeSubsystem.ClawState.OPEN),
                                        new WaitCommand(100)
                                )
                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new LiftPosCommand(obot, LiftSubsystem.LiftState.HIGH_BASKET),
                                        new WaitCommand(300),
                                        new BucketPosCommand(obot, LiftSubsystem.BucketState.BASKET)
                                ),
                                new SequentialCommandGroup(
                                        new PitchPosCommand(obot, IntakeSubsystem.PitchState.INTAKE),
                                        new WaitCommand(125),
                                        new ClawPosCommand(obot, IntakeSubsystem.ClawState.OPEN),
                                        new WaitCommand(125),
                                        new IntakePosCommand(obot, IntakeSubsystem.IntakeState.INTAKE)
                                )
                        ),
                        new WaitCommand(800),
                        new TriggerPosCommand(obot, LiftSubsystem.TriggerState.OPEN),
                        new WaitCommand(500),
                        //////////////////////PICK2///////////////////////////
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drive,pick2),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new BucketPosCommand(obot, LiftSubsystem.BucketState.TRANSFER),
                                        new WaitCommand(200),
                                        new LiftPosCommand(obot, LiftSubsystem.LiftState.HOME),
                                        new WaitCommand(600),
                                        new InstantCommand(() -> obot.Extendo.setMotionProfileTargetPosition(800))
                                )
                        ),
                        //new WaitCommand(600),
                        //new InstantCommand(() -> obot.Extendo.setMotionProfileTargetPosition(800)),
                        new WaitCommand(1300),
                        new ClawPosCommand(obot, IntakeSubsystem.ClawState.CLOSE),
                        new WaitCommand(600),
                        //new ExtendoPosCommand(obot, IntakeSubsystem.ExtendoState.HOME),
                        ////////////////////DEPOSIT2//////////////////////////
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drive,deposit2),
                                new SequentialCommandGroup(
                                        new ExtendoPosCommand(obot, IntakeSubsystem.ExtendoState.HOME),
                                        new WristPosCommand(obot, IntakeSubsystem.WristState.HOME),
                                        new WaitCommand(200),
                                        new IntakePosCommand(obot, IntakeSubsystem.IntakeState.TRANSFER),
                                        new WaitCommand(200),
                                        new PitchPosCommand(obot, IntakeSubsystem.PitchState.TRANSFER),
                                        new WaitCommand(400),
                                        new TriggerPosCommand(obot, LiftSubsystem.TriggerState.CLOSE),
                                        new WaitCommand(100),
                                        new ClawPosCommand(obot, IntakeSubsystem.ClawState.OPEN),
                                        new WaitCommand(200)
                                )
                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new LiftPosCommand(obot, LiftSubsystem.LiftState.HIGH_BASKET),
                                        new WaitCommand(300),
                                        new BucketPosCommand(obot, LiftSubsystem.BucketState.BASKET)
                                ),
                                new SequentialCommandGroup(
                                        new PitchPosCommand(obot, IntakeSubsystem.PitchState.INTAKE),
                                        new WaitCommand(125),
                                        new ClawPosCommand(obot, IntakeSubsystem.ClawState.OPEN),
                                        new WaitCommand(125),
                                        new IntakePosCommand(obot, IntakeSubsystem.IntakeState.INTAKE)
                                )
                        ),
                        new WaitCommand(800),
                        new TriggerPosCommand(obot, LiftSubsystem.TriggerState.OPEN),
                        new WaitCommand(600),
                        ///////////////////PICK3//////////////////////
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drive,pick3),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new BucketPosCommand(obot, LiftSubsystem.BucketState.TRANSFER),
                                        new WaitCommand(200),
                                        new LiftPosCommand(obot, LiftSubsystem.LiftState.HOME),
                                        new WaitCommand(1800),
                                        new InstantCommand(() -> obot.Extendo.setMotionProfileTargetPosition(850)),
                                        new WaitCommand(300),
                                        new WristPosCommand(obot, IntakeSubsystem.WristState.DIAGONAL_RIGHT)
                                )
                        ),
                        new WaitCommand(1000),
                        new ClawPosCommand(obot, IntakeSubsystem.ClawState.CLOSE),
                        new WaitCommand(700),
                        //////////////////DEPOSIT3//////////////////
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drive,deposit3),
                                new SequentialCommandGroup(
                                        new ExtendoPosCommand(obot, IntakeSubsystem.ExtendoState.HOME),
                                        new WristPosCommand(obot, IntakeSubsystem.WristState.HOME),
                                        new WaitCommand(200),
                                        new IntakePosCommand(obot, IntakeSubsystem.IntakeState.TRANSFER),
                                        new WaitCommand(200),
                                        new PitchPosCommand(obot, IntakeSubsystem.PitchState.TRANSFER),
                                        new WaitCommand(600),
                                        new TriggerPosCommand(obot, LiftSubsystem.TriggerState.CLOSE),
                                        new WaitCommand(100),
                                        new ClawPosCommand(obot, IntakeSubsystem.ClawState.OPEN),
                                        new WaitCommand(200)

                                )
                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new LiftPosCommand(obot, LiftSubsystem.LiftState.HIGH_BASKET),
                                        new WaitCommand(300),
                                        new BucketPosCommand(obot, LiftSubsystem.BucketState.BASKET)
                                ),
                                new SequentialCommandGroup(
                                        new PitchPosCommand(obot, IntakeSubsystem.PitchState.INTAKE),
                                        new WaitCommand(125),
                                        new ClawPosCommand(obot, IntakeSubsystem.ClawState.OPEN),
                                        new WaitCommand(125),
                                        new IntakePosCommand(obot, IntakeSubsystem.IntakeState.INTAKE)
                                )
                        ),
                        new WaitCommand(800),
                        new TriggerPosCommand(obot, LiftSubsystem.TriggerState.OPEN),
                        new WaitCommand(600),
                        new ParallelCommandGroup(
                                new WaitCommand(1200),
                                new BucketPosCommand(obot, LiftSubsystem.BucketState.TRANSFER),
                                new LiftPosCommand(obot, LiftSubsystem.LiftState.HOME),
                                new SequentialCommandGroup(
                                        new PitchPosCommand(obot, IntakeSubsystem.PitchState.INTAKE),
                                        new WaitCommand(125),
                                        new ClawPosCommand(obot, IntakeSubsystem.ClawState.OPEN),
                                        new WaitCommand(125),
                                        new IntakePosCommand(obot, IntakeSubsystem.IntakeState.INTAKE)
                                )
                        )
                )
        );

        while (opModeIsActive() && !isStopRequested()) {

            obot.clearBulkCache();
            commandScheduler1.getInstance().run();


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

            loopTime = (clock.seconds() - lastLoopTime) * 1000;

            telemetry.addData("Pos",obot.Lift.getPosition());
            telemetry.addData("TargetPos",obot.Lift.getTargetPosition());
            telemetry.addData("x", drive.getPoseEstimate().getX());
            telemetry.addData("y", drive.getPoseEstimate().getY());
            telemetry.addData("heading", drive.getPoseEstimate().getHeading());
            telemetry.addData("loop Time ms: ", loopTime);
            telemetry.update();


            lastLoopTime = clock.seconds();


        }
    }
}