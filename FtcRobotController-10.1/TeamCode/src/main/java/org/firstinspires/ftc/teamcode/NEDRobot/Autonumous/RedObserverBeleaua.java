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
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;
import org.firstinspires.ftc.teamcode.NEDRobot.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NEDRobot.trajectorysequence.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class RedObserverBeleaua extends LinearOpMode {

    public ElapsedTime imuTime;

    public static double p=0.009,i=0.000,d=0.0000;
    public PIDController controller;
    public static double velo=4000,accel=4000,decel=4000;

    private final Obot obot = Obot.getInstance();
    private FtcDashboard ftcDashboard;
    public SampleMecanumDrive drive;
    public int position;
    public VoltageSensor voltageSensor;
    public Pose2d POSE_START;
    public double IMU_FREQ=4;
    public IMU imu;

    public TrajectorySequence depositPreload,pick1,pick2,pick3;

    CommandScheduler commandScheduler;
    public static NanoClock clock = NanoClock.system();
    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        obot.init(hardwareMap,telemetry);
        obot.read();
        obot.Lift.setPID(0.01,0,0);

        CommandScheduler.getInstance().reset();
        drive = new SampleMecanumDrive(hardwareMap);
        imuTime=new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        commandScheduler.getInstance().reset();

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        telemetry.setMsTransmissionInterval(50);

        ftcDashboard = FtcDashboard.getInstance();

        depositPreload = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-36,-15,Math.toRadians(0)),
                        getVelocityConstraint(85,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(85))
                .build();

        pick1 = drive.trajectorySequenceBuilder(depositPreload.end())
                //.lineToLinearHeading(new Pose2d(-21,-5,Math.toRadians(0)),
                  //      getVelocityConstraint(75,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                    //    getAccelerationConstraint(75))
                .splineToLinearHeading(new Pose2d(-16,10,Math.toRadians(0)),Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(-16,32,Math.toRadians(0)),
                        getVelocityConstraint(65,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(65))
                .build();

        pick2 = drive.trajectorySequenceBuilder(pick1.end())
                .lineToLinearHeading(new Pose2d(-16,44,Math.toRadians(0)),
                        getVelocityConstraint(65,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(65))
                .build();

        pick3 = drive.trajectorySequenceBuilder(pick2.end())
                .lineToLinearHeading(new Pose2d(-13,42,Math.toRadians(-25)),
                        getVelocityConstraint(25,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(25))
                .build();


        double loopTime = 0;
        double imuAngle = 0.0;

        double lastLoopTime = 0;


        POSE_START = new Pose2d(0, 0, 0);
        //localizer
        drive.getLocalizer().setPoseEstimate(POSE_START);
        CommandScheduler.getInstance().reset();
        int ok=0;
        while (!isStarted() && !isStopRequested()) {
            if (ok == 0) {
                obot.periodic();
                obot.liftSubsystem.update(LiftSubsystem.LiftState.HOME);
                obot.Extendo.setMotionProfileTargetPosition(-10);
                obot.intakeSubsystem.update(IntakeSubsystem.IntakeState.AUTO);
                obot.intakeSubsystem.update(IntakeSubsystem.PitchState.INTAKE);
                obot.intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN);
                obot.intakeSubsystem.update(IntakeSubsystem.WristState.HOME);
                obot.liftSubsystem.update(LiftSubsystem.BucketState.TRANSFER);
                obot.liftSubsystem.update(LiftSubsystem.TriggerState.CLOSE_DEPOSIT);
                ok = 1;
            }
            if (ok <= 8) {
                CommandScheduler.getInstance().run();
                ok++;
            }

            drive.update();
            if (imuTime.seconds() > 1.0 / IMU_FREQ) {
                imuTime.reset();
                imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                Pose2d pose = drive.getPoseEstimate();
                drive.setPoseEstimate(new Pose2d(pose.getX(), pose.getY(), imuAngle));
            }


            telemetry.addLine("Obot is ready");
            telemetry.update();
        }

            waitForStart();
            commandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new SequentialCommandGroup(
                                    ///////////////PRELOAD///////////////
                                    new ParallelCommandGroup(
                                            new FollowTrajectoryCommand(drive,depositPreload),
                                            new LiftPosCommand(obot, LiftSubsystem.LiftState.LOW_BASKET),
                                            new BucketPosCommand(obot, LiftSubsystem.BucketState.CLAMP_DEPOSIT),
                                            new SequentialCommandGroup(
                                                    new WaitCommand(1700),
                                                    new TriggerPosCommand(obot, LiftSubsystem.TriggerState.OPEN)
                                            )
                                    ),
                                    ////////////PICK1////////////////
                                    new ParallelCommandGroup(
                                            new FollowTrajectoryCommand(drive,pick1),
                                            new ParallelCommandGroup(
                                                    new LiftPosCommand(obot, LiftSubsystem.LiftState.HOME),
                                                    new SequentialCommandGroup(
                                                            new WaitCommand(100),
                                                            new BucketPosCommand(obot, LiftSubsystem.BucketState.CLAMP)
                                                    )
                                            )
                                    ),

                                    new InstantCommand(() -> obot.Extendo.setMotionProfileTargetPosition(712)),
                                    new WaitCommand(500),
                                    new ClawPosCommand(obot, IntakeSubsystem.ClawState.CLOSE),
                                    new SequentialCommandGroup(
                                            new WristPosCommand(obot, IntakeSubsystem.WristState.HOME),
                                            new WaitCommand(200),
                                            new IntakePosCommand(obot, IntakeSubsystem.IntakeState.TRANSFER),
                                            new WaitCommand(200),
                                            new ParallelCommandGroup(
                                                    new PitchPosCommand(obot, IntakeSubsystem.PitchState.TRANSFER),
                                                    new ExtendoPosCommand(obot, IntakeSubsystem.ExtendoState.HOME)
                                            ),
                                            new WaitCommand(500),
                                            new TriggerPosCommand(obot, LiftSubsystem.TriggerState.CLOSE),
                                            new WaitCommand(300),
                                            new ClawPosCommand(obot, IntakeSubsystem.ClawState.OPEN),
                                            new WaitCommand(200),
                                            new BucketPosCommand(obot, LiftSubsystem.BucketState.CLAMP)
                                    ),
                                    new TriggerPosCommand(obot, LiftSubsystem.TriggerState.OPEN),
                                    ///////////PICK2//////////////
                                    new ParallelCommandGroup(
                                            new FollowTrajectoryCommand(drive,pick2),
                                            new ParallelCommandGroup(
                                                    new SequentialCommandGroup(
                                                            new PitchPosCommand(obot, IntakeSubsystem.PitchState.INTAKE),
                                                            new WaitCommand(125),
                                                            new ClawPosCommand(obot, IntakeSubsystem.ClawState.OPEN),
                                                            new WaitCommand(125),
                                                            new IntakePosCommand(obot, IntakeSubsystem.IntakeState.INTAKE)
                                                    ),
                                                    new LiftPosCommand(obot, LiftSubsystem.LiftState.HOME),
                                                    new TriggerPosCommand(obot, LiftSubsystem.TriggerState.OPEN),
                                                    new BucketPosCommand(obot, LiftSubsystem.BucketState.TRANSFER)
                                            )
                                    ),
                                    new InstantCommand(() -> obot.Extendo.setMotionProfileTargetPosition(730)),
                                    new WaitCommand(600),
                                    new ClawPosCommand(obot, IntakeSubsystem.ClawState.CLOSE),
                                    new SequentialCommandGroup(
                                            new WaitCommand(200),
                                            new ParallelCommandGroup(
                                                    new WristPosCommand(obot, IntakeSubsystem.WristState.HOME),
                                                    new IntakePosCommand(obot, IntakeSubsystem.IntakeState.TRANSFER),
                                                    new PitchPosCommand(obot, IntakeSubsystem.PitchState.TRANSFER),
                                                    new ExtendoPosCommand(obot, IntakeSubsystem.ExtendoState.HOME)
                                            ),
                                            new WaitCommand(500),
                                            new TriggerPosCommand(obot, LiftSubsystem.TriggerState.CLOSE),
                                            new WaitCommand(300),
                                            new ClawPosCommand(obot, IntakeSubsystem.ClawState.OPEN),
                                            new WaitCommand(200),
                                            new BucketPosCommand(obot, LiftSubsystem.BucketState.CLAMP)
                                    ),
                                    new WaitCommand(500),
                                    new TriggerPosCommand(obot, LiftSubsystem.TriggerState.OPEN),
                                    /////////////////PICK3//////////////////
                                    new ParallelCommandGroup(
                                            new FollowTrajectoryCommand(drive,pick3),
                                            new ParallelCommandGroup(
                                                    new SequentialCommandGroup(
                                                            new PitchPosCommand(obot, IntakeSubsystem.PitchState.INTAKE),
                                                            new WaitCommand(125),
                                                            new ClawPosCommand(obot, IntakeSubsystem.ClawState.OPEN),
                                                            new WaitCommand(125),
                                                            new IntakePosCommand(obot, IntakeSubsystem.IntakeState.INTAKE)
                                                    ),
                                                    new LiftPosCommand(obot, LiftSubsystem.LiftState.HOME),
                                                    new TriggerPosCommand(obot, LiftSubsystem.TriggerState.OPEN),
                                                    new BucketPosCommand(obot, LiftSubsystem.BucketState.TRANSFER)
                                            )
                                    ),
                                    new InstantCommand(() -> obot.Extendo.setMotionProfileTargetPosition(710)),
                                    new WaitCommand(600),
                                    new ClawPosCommand(obot, IntakeSubsystem.ClawState.CLOSE),
                                    new SequentialCommandGroup(
                                            new WaitCommand(200),
                                            new ParallelCommandGroup(
                                                    new WristPosCommand(obot, IntakeSubsystem.WristState.HOME),
                                                    new IntakePosCommand(obot, IntakeSubsystem.IntakeState.TRANSFER),
                                                    new PitchPosCommand(obot, IntakeSubsystem.PitchState.TRANSFER),
                                                    new ExtendoPosCommand(obot, IntakeSubsystem.ExtendoState.HOME)
                                            ),
                                            new WaitCommand(500),
                                            new TriggerPosCommand(obot, LiftSubsystem.TriggerState.CLOSE),
                                            new WaitCommand(300),
                                            new ClawPosCommand(obot, IntakeSubsystem.ClawState.OPEN),
                                            new WaitCommand(200),
                                            new BucketPosCommand(obot, LiftSubsystem.BucketState.CLAMP)
                                    ),
                                    new WaitCommand(500),
                                    new TriggerPosCommand(obot, LiftSubsystem.TriggerState.OPEN)


                            )
                    )
            );

            while (opModeIsActive() && !isStopRequested()) {

                obot.clearBulkCache();
                commandScheduler.getInstance().run();

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
