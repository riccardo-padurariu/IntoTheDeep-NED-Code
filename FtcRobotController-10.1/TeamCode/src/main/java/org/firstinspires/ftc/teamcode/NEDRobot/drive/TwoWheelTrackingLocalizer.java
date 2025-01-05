package org.firstinspires.ftc.teamcode.NEDRobot.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.NEDRobot.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->
 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class TwoWheelTrackingLocalizer extends TwoWheelCustom {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.688976378; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double X_MULTIPLIER = 0.9684131223;
    public static double Y_MULTIPLIER = 0.9933774834;//1.0465116279;

    public static double PARALLEL_X = -0.590551181;//-0.787402; // X is the up and down direction
    public static double PARALLEL_Y = -5.82677165;//6.02362205;//7.3031496; // Y is the strafe direction

    public static double PERPENDICULAR_X = 0;//-0.590551181;//0
    public static double PERPENDICULAR_Y = 6;//-5.23622047; //0.8070866

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private final Encoder parallelEncoder, perpendicularEncoder;

    private final SampleMecanumDrive drive;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive) {


        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightExtendoMotor"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightLiftMotor"));

        //parallelEncoder.setDirection(Encoder.Direction.REVERSE);
        //perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition() * X_MULTIPLIER),
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition() * Y_MULTIPLIER)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCorrectedVelocity() * X_MULTIPLIER),
                encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity() * Y_MULTIPLIER)
        );
    }

}