package org.firstinspires.ftc.teamcode.NEDRobot.Subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDMotor;
import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDMotorEncoder;
import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDServo;
import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.ProfileConstraints;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

@Config
public class Obot {

    public NEDServo claw;
    public NEDServo pitch_intake;
    public NEDServo rightIntake;
    public NEDServo leftIntake;
    public NEDServo wrist;

    public NEDServo trigger;
    public NEDServo leftBucket;
    public NEDServo rightBucket;

    public NEDServo hang;

    public DcMotor RightLiftMotor;
    public DcMotor LeftLiftMotor;
    public DcMotor LeftExtendoMotor;
    public DcMotor RightExtendoMotor;
    public NEDMotorEncoder extendoEncoder;
    public NEDMotorEncoder liftEncoder;


    public NEDMotor Lift;
    public NEDMotor Extendo;

    public IntakeSubsystem intakeSubsystem;
    public LiftSubsystem liftSubsystem;



    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    /**
     * Voltage timer and voltage value.
     */
    private ElapsedTime voltageTimer;
    private double voltage = 0.0;
    public double sensor=0.0;
    private VoltageSensor voltageSensor;

    /**
     * Singleton variables.
     */
    private static Obot instance = null;
    public boolean enabled;

    public List<LynxModule> modules;
    public DoubleSupplier volt;

    private ArrayList<NEDSubsystem> subsystems;

    /**
     * Creating the singleton the first time, instantiating.
     */
    public static Obot getInstance() {
        if (instance == null) {
            instance = new Obot();
        }
        instance.enabled = true;
        return instance;
    }

    /**
     * Created at the start of every OpMode.
     *
     * @param hardwareMap The HardwareMap of the robot, storing all hardware devices
     * @param telemetry Saved for later in the event FTC Dashboard used
     */
    public void init(final HardwareMap hardwareMap, final Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        this.subsystems = new ArrayList<>();

        modules = hardwareMap.getAll(LynxModule.class);
        modules.get(0).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        try {
            modules.get(1).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        } catch(Exception e) {}

        liftSubsystem = new LiftSubsystem();
        intakeSubsystem = new IntakeSubsystem();

        //OUTTAKE//


        LeftLiftMotor = hardwareMap.get(DcMotor.class,"leftLiftMotor");
        RightLiftMotor = hardwareMap.get(DcMotor.class,"rightLiftMotor");
        LeftExtendoMotor = hardwareMap.get(DcMotor.class,"leftExtendoMotor");
        RightExtendoMotor = hardwareMap.get(DcMotor.class,"rightExtendoMotor");
        LeftExtendoMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        extendoEncoder = new NEDMotorEncoder(new MotorEx(hardwareMap, "leftExtendoMotor").encoder);
        liftEncoder = new NEDMotorEncoder(new MotorEx(hardwareMap, "leftLiftMotor").encoder);



        rightBucket = new NEDServo(hardwareMap.get(Servo.class,"rightBucket"));
        leftBucket = new NEDServo(hardwareMap.get(Servo.class,"leftBucket"));
        trigger = new NEDServo(hardwareMap.get(Servo.class,"trigger"));
        leftBucket.setDirection(Servo.Direction.REVERSE);

        //GENERAL//
        hang = new NEDServo(hardwareMap.get(Servo.class,"hang"));

        //INTAKE//
        claw = new NEDServo(hardwareMap.get(Servo.class,"claw"));
        wrist = new NEDServo(hardwareMap.get(Servo.class,"wrist"));
        pitch_intake = new NEDServo(hardwareMap.get(Servo.class,"pitch_intake"));
        rightIntake = new NEDServo(hardwareMap.get(Servo.class,"rightIntake"));
        leftIntake = new NEDServo(hardwareMap.get(Servo.class,"leftIntake"));
        leftIntake.setDirection(Servo.Direction.REVERSE);







        voltageTimer = new ElapsedTime();

        this.Extendo = new NEDMotor(LeftExtendoMotor,RightExtendoMotor,extendoEncoder)
                .setPIDController(new PIDController(0.02,0.0,0.0005))
                .setMotionProfile(0,new ProfileConstraints(10000,10000,10000));

        this.Lift = new NEDMotor(LeftLiftMotor,RightLiftMotor,liftEncoder)
                .setPIDController(new PIDController(0.07, 0.0, 0.00035))
                .setMotionProfile(0, new ProfileConstraints(10000, 10000, 10000));


    }

    public void read() {
        liftSubsystem.read();
        intakeSubsystem.read();
    }

    public void write() {
        liftSubsystem.write();
        intakeSubsystem.write();
    }
    public void periodic() {
       /* if (voltageTimer.seconds() > 5) {
            voltageTimer.reset();
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        }
        volt=this::getVoltage;*/
        liftSubsystem.periodic();
        intakeSubsystem.periodic();

    }

    public void reset() {
    }

    public void clearBulkCache() {
        modules.get(0).clearBulkCache();
        modules.get(1).clearBulkCache();
    }

    public void addSubsystem(NEDSubsystem... subsystems) {
        this.subsystems.addAll(Arrays.asList(subsystems));
    }

    public double getVoltage() {
        return voltage;
    }


}