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
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.NEDRobot.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NEDRobot.trajectorysequence.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.trajectorysequence.TrajectorySequence;

/*@Autonomous
@Config
public class RedBasketBeleaua extends LinearOpMode {

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

    public TrajectorySequence Align1,Pick1,Deposit1;

    CommandScheduler commandScheduler;
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

        CommandScheduler.getInstance().reset();
        drive = new SampleMecanumDrive(hardwareMap);
        imuTime=new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        commandScheduler.getInstance().reset();

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        telemetry.setMsTransmissionInterval(50);

        ftcDashboard = FtcDashboard.getInstance();

        Align1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(0,37,Math.toRadians(0)))
                .build();

        Pick1 = drive.trajectorySequenceBuilder(Align1.end())
                .lineToLinearHeading(new Pose2d(7,37,Math.toRadians(0)),
                        getVelocityConstraint(7,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(7))
                .build();

        Deposit1 = drive.trajectorySequenceBuilder(Pick1.end())
                .lineToLinearHeading(new Pose2d(6,44,Math.toRadians(-45)))
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
            if(ok==0)
            {
                obot.Lift.periodic();
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new ClawPosCommand(obot, LiftSubsystem.ClawState.OPEN),
                                new InstantCommand(() -> obot.linkage.setPosition(0.29)),
                                new LiftPosCommand(obot, LiftSubsystem.LiftState.HOME),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> obot.arm.setTargetPosition(0)),
                                        new InstantCommand(() -> obot.arm.setPower(1)),
                                        new InstantCommand(() -> obot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION))
                                )
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

            waitForStart();
            commandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new SequentialCommandGroup(
                                    new FollowTrajectoryCommand(drive,Align1),
                                    new ArmPosCommand(obot, LiftSubsystem.ArmState.INTAKE),
                                    new WaitCommand(700),
                                    new LinkagePosCommand(obot, LiftSubsystem.LinkageState.EXTEND),
                                    new FollowTrajectoryCommand(drive,Pick1),
                                    new ClawPosCommand(obot, LiftSubsystem.ClawState.CLOSE),
                                    new WaitCommand(600),
                                    new ArmPosCommand(obot, LiftSubsystem.ArmState.TRANSFER),
                                    new WaitCommand(600),
                                    new LinkagePosCommand(obot, LiftSubsystem.LinkageState.HOME),
                                    new WaitCommand(600),
                                    new FollowTrajectoryCommand(drive,Deposit1),
                                    new WaitCommand(600),
                                    new ArmPosCommand(obot, LiftSubsystem.ArmState.BASKET),
                                    new WaitCommand(600),
                                    new InstantCommand(() -> obot.linkage.setPosition(0.1)),
                                    new WaitCommand(1500),
                                    new InstantCommand(() -> obot.claw.setPosition(0.2))
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
}*/
