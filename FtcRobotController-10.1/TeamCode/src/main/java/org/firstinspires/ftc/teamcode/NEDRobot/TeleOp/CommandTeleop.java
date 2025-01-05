package org.firstinspires.ftc.teamcode.NEDRobot.TeleOp;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
import org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.photoncore.Neutrino.RevColorSensor.RevColorSensorV3Ex;
import org.firstinspires.ftc.teamcode.photoncore.PhotonCore;
import org.w3c.dom.Comment;

import java.util.function.BooleanSupplier;

@Config
@TeleOp(name = "CommandTeleOp-Regionals", group = "National")

public class CommandTeleop extends CommandOpMode {
    private final Obot obotV2 = Obot.getInstance();

    int i = 0, j = 0;
    BooleanSupplier button;
    BooleanSupplier button1;
    double contor = 0;
    double sumLoop;
    private double loopTime = 0;
    private SampleMecanumDrive drive;
    double leftSen, rightSen;
    boolean Scoring = false;
    boolean Outtaking = false;
    boolean In_Intake = false, ClosedLeft = false, ClosedRight = false;
    boolean isHome = true;
    private GamepadEx GamepadEx1, GamepadEx2;
    ElapsedTime voltage_timer;
    VoltageSensor voltageSensor;
    double voltage;
    double transferPos = 340;
    double clampPos = 170;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        obotV2.init(hardwareMap,telemetry);
        obotV2.read();
        obotV2.periodic();
        obotV2.liftSubsystem.update(LiftSubsystem.LiftState.HOME);
        obotV2.intakeSubsystem.update(IntakeSubsystem.ExtendoState.HOME);
        obotV2.intakeSubsystem.update(IntakeSubsystem.IntakeState.INTAKE);
        obotV2.intakeSubsystem.update(IntakeSubsystem.PitchState.INTAKE);
        obotV2.intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN);
        obotV2.intakeSubsystem.update(IntakeSubsystem.WristState.HOME);
        obotV2.liftSubsystem.update(LiftSubsystem.BucketState.TRANSFER);
        obotV2.liftSubsystem.update(LiftSubsystem.TriggerState.OPEN);
        drive = new SampleMecanumDrive(hardwareMap);
        button = () -> gamepad1.start;
        button1 = () ->gamepad1.start;
        GamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx2 = new GamepadEx(gamepad2);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltage_timer = new ElapsedTime();

        GamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                        .whenPressed(
                                new SequentialCommandGroup(
                                        new ClawPosCommand(obotV2, IntakeSubsystem.ClawState.CLOSE),
                                        new WaitCommand(200),
                                        new WristPosCommand(obotV2, IntakeSubsystem.WristState.HOME),
                                        new WaitCommand(200),
                                        new IntakePosCommand(obotV2, IntakeSubsystem.IntakeState.TRANSFER),
                                        new WaitCommand(200),
                                        new ParallelCommandGroup(
                                                new PitchPosCommand(obotV2, IntakeSubsystem.PitchState.TRANSFER),
                                                new ExtendoPosCommand(obotV2, IntakeSubsystem.ExtendoState.HOME)
                                        ),
                                        new WaitCommand(500),
                                        new TriggerPosCommand(obotV2, LiftSubsystem.TriggerState.CLOSE),
                                        new WaitCommand(300),
                                        new ClawPosCommand(obotV2, IntakeSubsystem.ClawState.OPEN),
                                        new WaitCommand(200),
                                        new BucketPosCommand(obotV2, LiftSubsystem.BucketState.BASKET)
                                )
                        );
        GamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        new SequentialCommandGroup(
                                new ExtendoPosCommand(obotV2, IntakeSubsystem.ExtendoState.EXTEND)
                        )
                );
        GamepadEx1.getGamepadButton(GamepadKeys.Button.START)
                        .whenPressed(
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new PitchPosCommand(obotV2, IntakeSubsystem.PitchState.INTAKE),
                                                new WaitCommand(125),
                                                new ClawPosCommand(obotV2, IntakeSubsystem.ClawState.OPEN),
                                                new WaitCommand(125),
                                                new IntakePosCommand(obotV2, IntakeSubsystem.IntakeState.INTAKE)
                                        ),
                                        new LiftPosCommand(obotV2, LiftSubsystem.LiftState.HOME),
                                        new TriggerPosCommand(obotV2, LiftSubsystem.TriggerState.OPEN),
                                        new BucketPosCommand(obotV2, LiftSubsystem.BucketState.TRANSFER)
                                )
                        );


        GamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(
                        new SequentialCommandGroup(
                            new TriggerPosCommand(obotV2, LiftSubsystem.TriggerState.OPEN),
                            new WaitCommand(2000),
                            new BucketPosCommand(obotV2, LiftSubsystem.BucketState.TRANSFER)
                        )
                );

        GamepadEx1.getGamepadButton(GamepadKeys.Button.A)
                        .whenPressed(
                                new LiftPosCommand(obotV2, LiftSubsystem.LiftState.LOW_BASKET)
                        );
        GamepadEx1.getGamepadButton(GamepadKeys.Button.Y)
                        .whenPressed(
                                new LiftPosCommand(obotV2, LiftSubsystem.LiftState.HIGH_BASKET)
                                );
        GamepadEx1.getGamepadButton(GamepadKeys.Button.X)
                        .whenPressed(
                                //new ParallelCommandGroup(
                                  //      new InstantCommand(() -> obotV2.rightBucket.setPosition(toggle(obotV2.rightBucket.getPosition()))),
                                    //    new InstantCommand(() -> obotV2.leftBucket.setPosition(toggle(obotV2.leftBucket.getPosition())))
                                //)
                                    new BucketPosCommand(obotV2, LiftSubsystem.BucketState.CLAMP_DEPOSIT)
                        );
        GamepadEx1.getGamepadButton(GamepadKeys.Button.B)
                        .whenPressed(
                                new BucketPosCommand(obotV2, LiftSubsystem.BucketState.CLAMP)
                        );
        GamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                        .whenPressed(
                                new SequentialCommandGroup(
                                        new ClawPosCommand(obotV2, IntakeSubsystem.ClawState.CLOSE),
                                        new WaitCommand(300),
                                        new IntakePosCommand(obotV2, IntakeSubsystem.IntakeState.TRANSFER),
                                        new ParallelCommandGroup(
                                                new BucketPosCommand(obotV2, LiftSubsystem.BucketState.CLAMP),
                                                new ExtendoPosCommand(obotV2, IntakeSubsystem.ExtendoState.HOME),
                                                new SequentialCommandGroup(
                                                        new PitchPosCommand(obotV2, IntakeSubsystem.PitchState.DEPOSIT),
                                                        new WaitCommand(300),
                                                        new WristPosCommand(obotV2, IntakeSubsystem.WristState.HOME),
                                                        new WaitCommand(300),
                                                        new IntakePosCommand(obotV2, IntakeSubsystem.IntakeState.DEPOSIT)
                                                )
                                        )
                                )
                        );

        GamepadEx1.getGamepadButton(GamepadKeys.Button.BACK)
                        .whenPressed(
                                new SequentialCommandGroup(
                                new InstantCommand(() -> obotV2.intakeSubsystem.incrementWristPos(1)),
                                new InstantCommand(() -> obotV2.wrist.setPosition(obotV2.intakeSubsystem.getWristPos()))
                                )
                        );

        GamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                        .whenPressed(
                                new SequentialCommandGroup(
                                        new TriggerPosCommand(obotV2, LiftSubsystem.TriggerState.CLOSE_DEPOSIT),
                                        new WaitCommand(600),
                                        new BucketPosCommand(obotV2, LiftSubsystem.BucketState.CLAMP_DEPOSIT)
                                )
                        );

        PhotonCore.enable();

        obotV2.read();
    }
    @Override
    public void run() {
        obotV2.clearBulkCache();
        obotV2.read();


        if(gamepad1.left_trigger > 0){
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> obotV2.claw.setPosition(0.35)),
                            new WaitCommand(300),
                            new IntakePosCommand(obotV2, IntakeSubsystem.IntakeState.INTAKE),
                            new WaitCommand(300),
                            new PitchPosCommand(obotV2, IntakeSubsystem.PitchState.INTAKE),
                            new WaitCommand(300),
                            new WristPosCommand(obotV2, IntakeSubsystem.WristState.HOME),
                            new WaitCommand(300),
                            new ClawPosCommand(obotV2, IntakeSubsystem.ClawState.OPEN)
                    )
            );
        }



        if (Scoring) {
                drive.setWeightedDrivePower(
                        new Pose2d(dead(scale(GamepadEx1.getLeftY(), 0.6), 0) * 0.2,
                                dead(scale(-GamepadEx1.getLeftX(), 0.6), 0) * 0.2,
                                -GamepadEx1.getRightX() * 0.3
                        )
                );
            } else {
                drive.setWeightedDrivePower(
                        new Pose2d(dead(scale(GamepadEx1.getLeftY(), 0.6), 0) * (gamepad1.right_trigger > 0.5 ? 0.353 : 1),
                                dead(scale(-GamepadEx1.getLeftX(), 0.6), 0) * (gamepad1.right_trigger > 0.5 ? 0.4 : 1),
                                -GamepadEx1.getRightX() * (gamepad1.right_trigger > 0.5 ? 0.3 : 1)
                        )
                );
            }


        super.run();
        obotV2.periodic();
        /*telemetry.addData("Pos", obotV2.liftSubsystem.getExtendoHeight());
        telemetry.addData("In_Intake",In_Intake);
        telemetry.addData("LeftSen",leftSen);
        telemetry.addData("RightSen",rightSen);
        telemetry.addData("isHome",isHome);
        telemetry.addData("i",i);
        telemetry.addData("j",j);
        telemetry.addData("voltage",voltage);

        telemetry.addData("EXTENDO POS", obotV2.Extendo.getPosition());
        telemetry.addData("EXTENDO TARGET POS",obotV2.Extendo.getTargetPosition());
        telemetry.addData("EXTENDO POWER",obotV2.Extendo.getPower());
        telemetry.addData("pPower",obotV2.Extendo.pPower);
        telemetry.addData("pTargetPos",obotV2.Extendo.pTargetPosition);*/


        telemetry.addData("EXTENDO POS", obotV2.Lift.getPosition());
        telemetry.addData("EXTENDO TARGET POS",obotV2.Lift.getTargetPosition());
        telemetry.addData("EXTENDO POWER",obotV2.Lift.getPower());
        telemetry.addData("bucket pos", obotV2.rightBucket.getPosition());
        telemetry.addData("bucket pos", obotV2.leftBucket.getPosition());
        double loop = System.nanoTime();
        contor++;
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        sumLoop+=1000000000 / (loop - loopTime);
            loopTime = loop;
        telemetry.update();
        obotV2.write();

    }
    @Override
    public void reset() {
           CommandScheduler.getInstance().reset();
        obotV2.reset();
    }

    public double scale(double x, double k) {
        return (1 - k) * x + k * x * x * x;
    }

    public double dead(double x, double k) {
        return Math.abs(x) > k ? x : 0;
    }
    public double toggle(double servoPos){
        if(servoPos*360 == transferPos)
            return clampPos;
        return transferPos;
    }
}

