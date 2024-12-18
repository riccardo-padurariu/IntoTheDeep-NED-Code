package org.firstinspires.ftc.teamcode.NEDRobot.TeleOp;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
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

import java.util.function.BooleanSupplier;

@Config
@TeleOp(name = "CommandTeleOpOffSeason", group = "National")

public class CommandTeleop extends CommandOpMode {
    private final Obot obotV2 = Obot.getInstance();

    int i = 0, j = 0;
    BooleanSupplier button;
    BooleanSupplier button1;
    double contor = 0;
    double sumLoop;
    public RevColorSensorV3Ex intakeSensorLeft;
    public RevColorSensorV3Ex intakeSensorRight;

    private double distance_to_intake = 1.2;
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


    @Override
    public void initialize() {
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //intakeSensorLeft = hardwareMap.get(RevColorSensorV3Ex.class,"intakeSensorLeft");
        //intakeSensorLeft.setUpdateRate(RevColorSensorV3Ex.UPDATE_RATE.HIGH_SPEED);
        //intakeSensorRight = hardwareMap.get(RevColorSensorV3Ex.class,"intakeSensorRight");
        //intakeSensorRight.setUpdateRate(RevColorSensorV3Ex.UPDATE_RATE.HIGH_SPEED);
        CommandScheduler.getInstance().reset();
        obotV2.init(hardwareMap,telemetry);
        obotV2.read();
        obotV2.periodic();
        obotV2.liftSubsystem.update(LiftSubsystem.LiftState.HOME);
        //obotV2.intakeSubsystem.update(IntakeSubsystem.ExtendoState.HOME);
        //obotV2.LeftExtendoMotor.setTargetPosition(-10);
        //obotV2.LeftExtendoMotor.setPower(1);
        //obotV2.LeftExtendoMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //obotV2.RightExtendoMotor.setTargetPosition(-10);
        ///obotV2.RightExtendoMotor.setPower(1);
        //obotV2.RightExtendoMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        obotV2.intakeSubsystem.update(IntakeSubsystem.ExtendoState.HOME);
        obotV2.intakeSubsystem.update(IntakeSubsystem.IntakeState.INTAKE);
        obotV2.intakeSubsystem.update(IntakeSubsystem.PitchState.INTAKE);
        obotV2.intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN);
        //obotV2.claw.setPosition(1);
        obotV2.intakeSubsystem.update(IntakeSubsystem.WristState.HOME);
        obotV2.liftSubsystem.update(LiftSubsystem.BucketState.BASKET);
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
                                        new WaitCommand(1000),
                                        new IntakePosCommand(obotV2, IntakeSubsystem.IntakeState.TRANSFER),
                                        new WaitCommand(1000),
                                        new InstantCommand(() -> obotV2.LeftExtendoMotor.setTargetPosition(-100)),
                                        new InstantCommand(() -> obotV2.LeftExtendoMotor.setPower(1)),
                                        new InstantCommand(() -> obotV2.LeftExtendoMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION)),
                                        new InstantCommand(() -> obotV2.RightExtendoMotor.setTargetPosition(-100)),
                                        new InstantCommand(() -> obotV2.RightExtendoMotor.setPower(1)),
                                        new InstantCommand(() -> obotV2.RightExtendoMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION)),
                                        new WaitCommand(1000),
                                        new WristPosCommand(obotV2, IntakeSubsystem.WristState.TRANSFER),
                                        new WaitCommand(1000),
                                        new PitchPosCommand(obotV2, IntakeSubsystem.PitchState.TRANSFER),
                                        new WaitCommand(1000),
                                        new BucketPosCommand(obotV2, LiftSubsystem.BucketState.TRANSFER),
                                        new WaitCommand(1000),
                                        new TriggerPosCommand(obotV2, LiftSubsystem.TriggerState.CLOSE),
                                        new WaitCommand(1000),
                                        new ClawPosCommand(obotV2, IntakeSubsystem.ClawState.OPEN)
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
                                new SequentialCommandGroup(
                                        new ClawPosCommand(obotV2, IntakeSubsystem.ClawState.CLOSE),
                                        new WaitCommand(1000),
                                        new PitchPosCommand(obotV2, IntakeSubsystem.PitchState.INTAKE),
                                        new WaitCommand(1000),
                                        new WristPosCommand(obotV2, IntakeSubsystem.WristState.HOME),
                                        new WaitCommand(1000),
                                        new ClawPosCommand(obotV2, IntakeSubsystem.ClawState.OPEN),
                                        new WaitCommand(1000),
                                        new IntakePosCommand(obotV2, IntakeSubsystem.IntakeState.INTAKE)
                                )
                        );

        GamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                        .whenPressed(
                                new SequentialCommandGroup(
                                        new BucketPosCommand(obotV2, LiftSubsystem.BucketState.BASKET)
                                )
                        );

        GamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                        .whenPressed(
                                new TriggerPosCommand(obotV2, LiftSubsystem.TriggerState.CLOSE)
                        );

        GamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(
                        new TriggerPosCommand(obotV2, LiftSubsystem.TriggerState.OPEN)
                );

        GamepadEx1.getGamepadButton(GamepadKeys.Button.X)
                        .whenPressed(
                                new LiftPosCommand(obotV2, LiftSubsystem.LiftState.LOW_BASKET)
                        );


        PhotonCore.enable();

        obotV2.read();
    }
    @Override
    public void run() {
        obotV2.clearBulkCache();
        obotV2.read();

        /*if(gamepad1.right_bumper){
            obotV2.LeftExtendoMotor.setTargetPosition(1050);
            obotV2.LeftExtendoMotor.setPower(1);
            obotV2.LeftExtendoMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            obotV2.RightExtendoMotor.setTargetPosition(1050);
            obotV2.RightExtendoMotor.setPower(1);
            obotV2.RightExtendoMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }*/


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
        if(In_Intake) {
            Outtaking=false;
            leftSen=intakeSensorLeft.getDistance(DistanceUnit.CM);
            rightSen=intakeSensorRight.getDistance(DistanceUnit.CM);
            if (leftSen<distance_to_intake && !ClosedLeft) {
                i++;
            }
            if(i==4)
            {
                i=0;
                ClosedLeft=true;
            }
            if (rightSen<distance_to_intake && !ClosedRight) {
                j++;
            }
            if(j==4)
            {
                j=0;
                ClosedRight=true;
            }
            if (ClosedRight && ClosedLeft) {
                ClosedRight=false;
                ClosedLeft=false;
                In_Intake = false;
                //CommandScheduler.getInstance().schedule(new TransferCommand(obotV2));
                gamepad1.rumble(1000);
                isHome = true;
            }
        }
        else
        {
            i=0;j=0;ClosedLeft=false;ClosedRight=false;
        }
        //gamepad1.rumble(200); -> cod pentru a vibra controller ul

        super.run();
        obotV2.periodic();
        telemetry.addData("Pos", obotV2.liftSubsystem.getLiftHeight());
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
        telemetry.addData("pTargetPos",obotV2.Extendo.pTargetPosition);



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
}

