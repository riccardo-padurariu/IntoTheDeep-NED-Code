package org.firstinspires.ftc.teamcode.NEDRobot.Hardware;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;
import org.firstinspires.ftc.teamcode.NEDRobot.util.MathUtil;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.ProfileConstraints;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.ProfileState;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;

public class NEDMotor {
    public enum FeedforwardMode {
        NONE,
        CONSTANT,
        ANGLE_BASED,
        ANGLE_BASED_SIN
    }

    private final Map<String, HardwareDevice> devices = new HashMap<>();
    private AsymmetricMotionProfile profile;
    private ProfileConstraints constraints;
    private ProfileState state;
    private PIDController controller;
    public ElapsedTime timer;

    private double position = 0.0;
    private double targetPosition = 0.0;
    public double pTargetPosition = 0.0;
    private double overallTargetPosition = 0.0;

    public double pPower = 0.0;

    private double power = 0.0;
    private double tolerance = 0.0;
    private double feedforwardMin = 0.0;
    private double feedforwardMax = 0.0;
    private double currentFeedforward = 0.0;
    private double targetPositionOffset = 0.0;
    private double offset = 0.0;
    public double ok=0;
    private DoubleSupplier voltage;
    private Obot obot= Obot.getInstance();
    private boolean reached = false;
    private boolean floating = false;


    private FeedforwardMode mode = FeedforwardMode.NONE;

    /**
     * Actuator constructor with varargs HardwareDevice parameter
     *
     * @param devices
     */
    public NEDMotor(HardwareDevice... devices) {
        int i=1;
        for (HardwareDevice device : devices) {
            this.devices.put(device.getDeviceName() + " " + i++, device);
        }
        read();
    }

    /**
     * Reads every given hardware device containing either AnalogEncoder or Encoder object.
     * Will then store this, and terminate the loop, because there is only a need for one value in
     * a given actuation group.
     */
    public void read() {
        for (HardwareDevice device : devices.values()) {
            if (device instanceof NEDAnalogEncoder) {
                this.position = ((NEDAnalogEncoder) device).getCurrentPosition() + offset;
                return;
            } else if (device instanceof NEDMotorEncoder) {
                this.position = (int) (((NEDMotorEncoder) device).getPosition() + offset);
                return;
            }
        }
        this.position=0.0;
    }

    /**
     * Performs arithmetic with the AsymmetricMotionProfile and PIDController.
     * Stores a boolean representing whether or not the actuator group is within
     * some tolerance given by a specified value.
     */
    public void periodic() {
        if (timer == null) {
            timer = new ElapsedTime();
        }

        if (profile != null) {
            this.state = profile.calculate(timer.time());
            this.targetPosition = state.x + targetPositionOffset;
        }

        if (controller != null) {
            this.power = controller.calculate(position, targetPosition + targetPositionOffset);

            switch (mode) {
                case CONSTANT:
                    this.power += currentFeedforward * Math.signum((targetPosition + targetPositionOffset) - position);
                    break;
                case ANGLE_BASED:
                    this.power += Math.cos(position) * currentFeedforward;
                    break;
                case ANGLE_BASED_SIN:
                    this.power += Math.sin(position) * currentFeedforward;
                    break;
                default:
            }
            this.power = MathUtil.clamp(power, -1, 1);
        }

        this.reached = Math.abs((targetPosition + targetPositionOffset) - position) < tolerance;
    }

    /**
     * For cohesiveness in our program's sequence, writes back all of the new
     * values calculated and saved. Runs different methods based on the given actuation group.
     */
    public void write() {
        if (Math.abs(targetPosition - pTargetPosition) > 0.005 ||
                Math.abs(power - pPower) > 0.005) {
            for (HardwareDevice device : devices.values()) {
                if (device instanceof DcMotor) {
                    //double correction = 1.0;
                    //if (voltage != null) correction = 12.0 / voltage.getAsDouble();
                    ((DcMotor) device).setPower(power /* correction*/);
                }
            }
        }
        pTargetPosition = targetPosition;
        pPower = power;
    }

    /**
     * Will set the target position for the actuator group. In the case of a motion profile
     * being used, the profile will be reset and created again with the new target position.
     * Otherwise, a PIDController will be used in the periodic() method above.
     *
     * @param targetPosition
     */
    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    public void setFloat(boolean f) {
        this.floating = f;
    }
    public void setOffset(double offset) {
        this.offset = offset;
    }
    public void setTargetPositionOffset(double offset) {
        this.targetPositionOffset = offset;
    }

    public void setMotionProfileTargetPosition(double targetPosition) {
        this.profile = new AsymmetricMotionProfile(getTargetPosition(), targetPosition, constraints);
        this.timer.reset();
    }
    public NEDMotor setVoltageSupplier(DoubleSupplier voltage) {
        this.voltage = voltage;
        return this;
    }

    public void setMotionProfileAdditionPosition(double addition) {
        this.profile = null;
        this.targetPosition=getPosition()+addition;
    }
    /**
     * Saves the value passed in for the new motion profile.
     *
     //     * @param profile The new asymmetrical motion profile
     * @return
     */
    public NEDMotor setMotionProfile(double targetPosition, ProfileConstraints constraints) {
        this.constraints = constraints;
        this.profile = new AsymmetricMotionProfile(position, targetPosition, constraints);
        return this;
    }

    /**
     * Saves the value passed in for the new PID controller.
     *
     * @param controller
     * @return
     */
    public NEDMotor setPIDController(PIDController controller) {
        this.controller = controller;
        return this;
    }

    /**
     * Creates a new PIDController object based on the coefficients passed in.
     *
     * @param p Proportional constant
     * @param i Integral Constant
     * @param d Derivative Constant
     * @return
     */
    public NEDMotor setPID(double p, double i, double d) {
        if (controller == null) {
            this.controller = new PIDController(p, i, d);
        } else {
            this.controller.setPID(p, i, d);
        }

        return this;
    }

    public NEDMotor setFeedforward(FeedforwardMode mode, double feedforward) {
        this.mode = mode;
        this.feedforwardMin = feedforward;
        this.currentFeedforward = feedforwardMin;
        return this;
    }

    public NEDMotor setFeedforward(FeedforwardMode mode, double feedforwardMin, double feedforwardMax) {
        this.mode = mode;
        this.feedforwardMin = feedforwardMin;
        this.feedforwardMax = feedforwardMax;
        this.currentFeedforward = feedforwardMin;
        return this;
    }

    /**
     * Sets the allowed error tolerance for the actuation group to be considered
     * "close enough" within the target position.
     *
     * @param tolerance
     * @return
     */
    public NEDMotor setErrorTolerance(double tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    public void updateConstraints(ProfileConstraints constraints) {
        this.constraints = constraints;
//        setMotionProfile(new AsymmetricMotionProfile(position, targetPosition, constraints));
//        this.constr
    }

    public void updatePID(double P, double I, double D) {
        this.controller.setPID(P, I, D);
    }

    public void updateFeedforward(double percentage) {
        this.currentFeedforward = feedforwardMin + (feedforwardMax - feedforwardMin) * percentage;
    }

    /**
     * Gets the value read by the actuation group.
     *
     * @return double
     */
    public double getPosition() {
        return position;
    }

    /**
     * Gets the current target position for the actuation group.
     * @return double
     */
    public double getTargetPosition() {
        return targetPosition;
    }

    public double getPower() {
        return power;
    }

    public double getCurrentFeedforward() {
        return currentFeedforward;
    }

    /**
     * Gets any given HardwareDevice within this actuator group.
     *
     * @param deviceName The given HardwareMap name specified in the config,
     *                   or specified at object creation.
     * @return HardwareDevice object
     */
    public HardwareDevice getDevice(String deviceName) {
        return this.devices.get(deviceName);
    }

    /**
     * Returns a list of all given HardwareDevices.
     *
     * @return
     */
    public List<HardwareDevice> getDevices() {
        return new ArrayList<>(devices.values());
    }

    public ProfileState getState() {
        return this.state;
    }

    public ProfileConstraints getConstraints() {
        return this.constraints;
    }

    /**
     * Returns whether or not the given actuation group is within error
     * tolerance of the final position.
     *
     * @return
     */
    public boolean hasReached() {
        return this.reached;
    }
}