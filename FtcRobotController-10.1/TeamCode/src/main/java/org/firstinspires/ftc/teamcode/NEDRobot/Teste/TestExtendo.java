package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDMotorEncoder;
import org.firstinspires.ftc.teamcode.NEDRobot.util.MathUtil;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.ProfileConstraints;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.ProfileState;

@Config
@TeleOp(name="Test-Extendo")
public class TestExtendo extends LinearOpMode {
    public DcMotorEx LeftExtendoMotor;
    public DcMotorEx RightExtendoMotor;

    public NEDMotorEncoder intakeEncoder;
    private double position;
    private ElapsedTime voltageTimer;
    private VoltageSensor voltageSensor;
    private double voltage = 0.0;
    public static double targetposition,realtarget;
    private double pTargetPosition = 0.0;
    private double pPower =0.0;
    public static double p=0.04,i=0.000,d=0.0004;
    public PIDController controller;
    private double loopTime = 0.0;
    private ElapsedTime timer;
    private AsymmetricMotionProfile profile;
    private ProfileConstraints constraints;
    private ProfileState state;
    public static double velo=10000,accel=10000,decel=10000;
    private double power;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        LeftExtendoMotor = hardwareMap.get(DcMotorEx.class,"leftExtendoMotor");
        intakeEncoder = new NEDMotorEncoder(new MotorEx(hardwareMap, "leftExtendoMotor").encoder);
        RightExtendoMotor = hardwareMap.get(DcMotorEx.class,"rightExtendoMotor");
        LeftExtendoMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDController(p,i,d);
        controller.setPID(p,i,d);
        constraints = new ProfileConstraints(velo,accel,decel);
        profile = new AsymmetricMotionProfile(0,0,constraints);
        timer = new ElapsedTime();
        state =new ProfileState();
        voltageTimer = new ElapsedTime();
        voltageTimer.reset();
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltage = voltageSensor.getVoltage();
        intakeEncoder.encoder.reset();
        waitForStart();
        while(!isStopRequested() && opModeIsActive())
        {
            ///Read
            position=intakeEncoder.getPosition();

            if(p!=controller.getP() || i!=controller.getI() || d!=controller.getD())
                controller.setPID(p,i,d);
            if(gamepad1.a)
            {
                constraints.accel=accel;
                constraints.decel=decel;
                constraints.velo=velo;
                realtarget=targetposition;
                profile = new AsymmetricMotionProfile(position,realtarget,constraints);
                timer.reset();
            }
            ////Periodic

            state = profile.calculate(timer.time());
            realtarget = state.x;
            power=controller.calculate(position,realtarget);
            power= MathUtil.clamp(power,-1,1);
            if (voltageTimer.seconds() > 5) {
                voltage = voltageSensor.getVoltage();
                voltageTimer.reset();
            }
            ///Write
            if (Math.abs(realtarget - pTargetPosition) > 0.005 ||
                    Math.abs(power - pPower) > 0.005) {
                double correction = 1.0;
                if (voltage !=0) correction = 12.0 / voltage;
                LeftExtendoMotor.setPower(power*correction);
                RightExtendoMotor.setPower(power*correction);
            }
            pTargetPosition = realtarget;
            pPower = power;


            telemetry.addData("Position",position);
            telemetry.addData("TargetPosition",realtarget);
            telemetry.addData("Current", LeftExtendoMotor.getCurrent(CurrentUnit.AMPS));
            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();
        }
    }
    public double getTargetPosition() {
        return targetposition;
    }

}
