package org.firstinspires.ftc.teamcode.NEDRobot.Autonumous;


import static org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive.getVelocityConstraint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.drive.Drive;
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
public class RedObserverBeleauaSasiu extends LinearOpMode {

    public ElapsedTime imuTime;

    public static double p=0.0024,i=0.000,d=0.0000;
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

    public TrajectorySequence depositPreload,pick1,pick2,pick3,pickHuman1,pickHuman2,deposit1,deposit2,pickHuman3,deposit3;

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
        obot.Extendo.setPID(p,i,0.000);
        obot.Lift.setPID(0.004,i,0.000);

        CommandScheduler.getInstance().reset();
        drive = new SampleMecanumDrive(hardwareMap);
        imuTime=new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        commandScheduler.getInstance().reset();

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        telemetry.setMsTransmissionInterval(50);

        ftcDashboard = FtcDashboard.getInstance();

        depositPreload = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(34,0,Math.toRadians(0)),
                        getVelocityConstraint(90,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(90))
                .build();

        pick1 = drive.trajectorySequenceBuilder(depositPreload.end())
                //.lineToLinearHeading(new Pose2d(-21,-5,Math.toRadians(0)),
                //      getVelocityConstraint(75,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                //    getAccelerationConstraint(75))
                .lineToLinearHeading(new Pose2d(-25,-10,Math.toRadians(0)),
                        getVelocityConstraint(110,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(110))
                .lineToLinearHeading(new Pose2d(-25,20,Math.toRadians(0)),
                        getVelocityConstraint(110,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(110))
                .lineToLinearHeading(new Pose2d(-50,20,Math.toRadians(0)),
                        getVelocityConstraint(110,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(110))
                .lineToLinearHeading(new Pose2d(-50,30,Math.toRadians(0)),
                        getVelocityConstraint(110,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(110))
                .lineToLinearHeading(new Pose2d(-13,30,Math.toRadians(0)),
                        getVelocityConstraint(110,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(110))
                .lineToLinearHeading(new Pose2d(-50,20,Math.toRadians(0)),
                        getVelocityConstraint(110,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(110))
                .lineToLinearHeading(new Pose2d(-50,40,Math.toRadians(0)),
                        getVelocityConstraint(110,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(110))
                .lineToLinearHeading(new Pose2d(-13,30,Math.toRadians(0)),
                        getVelocityConstraint(110,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(110))
                .build();

        pick2 = drive.trajectorySequenceBuilder(pick1.end())
                .lineToLinearHeading(new Pose2d(-13,45,Math.toRadians(0)),
                        getVelocityConstraint(100,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(100))
                .build();

//        pick3 = drive.trajectorySequenceBuilder(pick2.end())
//                .lineToLinearHeading(new Pose2d(-13,42,Math.toRadians(-25)),
//                        getVelocityConstraint(25,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
//                        getAccelerationConstraint(25))
//                .build();

        pickHuman1 = drive.trajectorySequenceBuilder(pick1.end())
                .lineToLinearHeading(new Pose2d(-13,20,Math.toRadians(0)),
                        getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(100))
                .lineToLinearHeading(new Pose2d(-7.5,20,Math.toRadians(0)),
                        getVelocityConstraint(100,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(100))
                .build();

        deposit1 = drive.trajectorySequenceBuilder(pickHuman1.end())
                .lineToLinearHeading(new Pose2d(-36,-19,Math.toRadians(0)),
                        getVelocityConstraint(100,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(100))
                .build();

        pickHuman2 = drive.trajectorySequenceBuilder(deposit1.end())
                .lineToLinearHeading(new Pose2d(-9.5,20,Math.toRadians(0)),
                        getVelocityConstraint(100,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(100))
                .build();

        deposit2 = drive.trajectorySequenceBuilder(pickHuman2.end())
                .lineToLinearHeading(new Pose2d(-36,-16,Math.toRadians(0)),
                        getVelocityConstraint(100,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(100))
                .build();

        pickHuman3 = drive.trajectorySequenceBuilder(deposit2.end())
                .lineToLinearHeading(new Pose2d(-10,22,Math.toRadians(0)),
                        getVelocityConstraint(100,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(100))
                .build();

        deposit3 = drive.trajectorySequenceBuilder(pickHuman3.end())
                .lineToLinearHeading(new Pose2d(-40,-13,Math.toRadians(0)),
                        getVelocityConstraint(100,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(100))
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
                obot.intakeSubsystem.update(IntakeSubsystem.IntakeState.INTAKE);
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


            telemetry.addLine("Obot is ready to go vrum vrum");
            telemetry.update();
        }

        waitForStart();
        commandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drive,depositPreload),
                                new SequentialCommandGroup(
                                        new ExtendoPosCommand(obot, IntakeSubsystem.ExtendoState.EXTEND),
                                        new WaitCommand(1000),
                                        new ClawPosCommand(obot, IntakeSubsystem.ClawState.CLOSE),
                                        new WaitCommand(1000),
                                        new IntakePosCommand(obot, IntakeSubsystem.IntakeState.TRANSFER)
                                )
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
