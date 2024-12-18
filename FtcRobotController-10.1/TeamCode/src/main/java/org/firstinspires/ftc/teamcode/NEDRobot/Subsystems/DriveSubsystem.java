package org.firstinspires.ftc.teamcode.NEDRobot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDSubsystem;

@Config
public class DriveSubsystem extends NEDSubsystem {
    private double[] ws = new double[4];
    double[] pws = new double[4];
    public  DcMotorEx leftFront;
    public  DcMotorEx rightRear;
    public  DcMotorEx rightFront;
    public  DcMotorEx leftRear;

    public DriveSubsystem(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "FS");//SD
        leftRear = hardwareMap.get(DcMotorEx.class, "SS");//FD
        rightRear = hardwareMap.get(DcMotorEx.class, "SD");//FS
        rightFront = hardwareMap.get(DcMotorEx.class, "FD");//SS


        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void periodic() {
    }

    @Override
    public void read() {
        // Nothing here
    }
    public void control(double forwardSpeed,double strafeSpeed,double turnSpeed)
    {

        forwardSpeed= Range.clip(forwardSpeed,-1,1);
        strafeSpeed= Range.clip(strafeSpeed,-1,1);
        turnSpeed= Range.clip(turnSpeed,-1,1);

        ws[RobotDrive.MotorType.kFrontLeft.value] = forwardSpeed + strafeSpeed + turnSpeed;
        ws[RobotDrive.MotorType.kFrontRight.value] = forwardSpeed - strafeSpeed - turnSpeed;
        ws[RobotDrive.MotorType.kBackLeft.value] = forwardSpeed - strafeSpeed + turnSpeed;
        ws[RobotDrive.MotorType.kBackRight.value] = (forwardSpeed + strafeSpeed - turnSpeed);

        double max = 1;
        for (double wsw : ws) max = Math.max(max, Math.abs(wsw));

        if (Math.abs(max) > 1) {
            ws[RobotDrive.MotorType.kFrontLeft.value] /= max;
            ws[RobotDrive.MotorType.kFrontRight.value] /= max;
            ws[RobotDrive.MotorType.kBackLeft.value] /= max;
            ws[RobotDrive.MotorType.kBackRight.value] /= max;
        }


    }
    @Override
    public void write() {
        if (Math.abs(ws[0] - pws[0]) > 0.005) leftFront.setPower(ws[0]);
        if (Math.abs(ws[1] - pws[1]) > 0.005) rightFront.setPower(ws[1]);
        if (Math.abs(ws[2] - pws[2]) > 0.005) leftRear.setPower(ws[2]);
        if (Math.abs(ws[3] - pws[3]) > 0.005) rightRear.setPower(ws[3]);

        pws[0] = ws[0];
        pws[1] = ws[1];
        pws[2] = ws[2];
        pws[3] = ws[3];
    }

    @Override
    public void reset() {
    }

}