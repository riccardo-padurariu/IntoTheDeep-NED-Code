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
@TeleOp(name="Test-Lift")
public class TestLift extends LinearOpMode {
    public DcMotorEx LeftLiftMotor;
    public DcMotorEx RightLiftMotor;
    public NEDMotorEncoder liftEncoder;
    private double position;
    private ElapsedTime voltageTimer;
    private VoltageSensor voltageSensor;
    private double voltage = 0.0;
    public static double targetposition;
    public double realtarget;
    private double pTargetPosition = 0.0;
    private double pPower =0.0;
    public static double p=0.03,i=0.000,d=0.00025;
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
        LeftLiftMotor = hardwareMap.get(DcMotorEx.class,"leftLiftMotor");
        RightLiftMotor = hardwareMap.get(DcMotorEx.class,"rightLiftMotor");
        LeftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftEncoder = new NEDMotorEncoder(new MotorEx(hardwareMap, "leftLiftMotor").encoder);
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
        liftEncoder.encoder.reset();
        waitForStart();
        while(!isStopRequested() && opModeIsActive())
        {
            ///Read
            //position=liftEncoder.getPosition();
            position = LeftLiftMotor.getCurrentPosition();

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
            if (Math.abs(targetposition - pTargetPosition) > 0.005 ||
                    Math.abs(power - pPower) > 0.005) {
                double correction = 1.0;
                if (voltage !=0) correction = 12.0 / voltage;
                LeftLiftMotor.setPower(power*correction);
                RightLiftMotor.setPower(power*correction);
            }
            pTargetPosition = targetposition;
            pPower = power;


            telemetry.addData("Position",position);
            telemetry.addData("TargetPosition",realtarget);
            telemetry.addData("Current",LeftLiftMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Current",RightLiftMotor.getCurrent(CurrentUnit.AMPS));
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
