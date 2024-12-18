package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import static org.firstinspires.ftc.teamcode.NEDRobot.drive.DriveConstants.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.NEDRobot.drive.DriveConstants.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.NEDRobot.drive.DriveConstants.WHEEL_RADIUS;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDMotor;
import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDMotorEncoder;

@TeleOp
public class TestEncoderOdo extends OpMode {
    NEDMotorEncoder encoder;
    NEDMotorEncoder encoder1;
    NEDMotorEncoder encoder2;
    NEDMotorEncoder encoder3;
    NEDMotor motor;
    @Override
    public void init() {
        encoder = new NEDMotorEncoder(new MotorEx(hardwareMap, "FS").encoder);
        encoder1 = new NEDMotorEncoder(new MotorEx(hardwareMap, "SS").encoder);
        encoder2 = new NEDMotorEncoder(new MotorEx(hardwareMap, "active").encoder);
        //encoder3 = new NEDMotorEncoder(new MotorEx(hardwareMap, "leftLiftMotor").encoder);

    }
    @Override
    public void loop() {
        telemetry.addData("FS",encoder.getPosition());
        telemetry.addData("SS",encoderTicksToInches(encoder1.getPosition()));
        telemetry.addData("active",encoderTicksToInches(encoder2.getPosition()));
        telemetry.update();
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
}
