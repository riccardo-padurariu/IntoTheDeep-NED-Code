package org.firstinspires.ftc.teamcode.NEDRobot.Subsystems;

import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.util.MathUtil;

public class IntakeSubsystem extends NEDSubsystem {
    private final Obot obot = Obot.getInstance();

    public enum IntakeState{
        INTAKE,
        TRANSFER,
        DEPOSIT
    }

    public enum ExtendoState{
        HOME,
        EXTEND
    }

    public enum PitchState{
        INTAKE,
        TRANSFER,
        DEPOSIT
    }

    public enum ClawState{
        OPEN,
        CLOSE
    }

    public enum WristState{
        HOME,
        DIAGONAL_LEFT,
        DIAGONAL_RIGHT,
        TRANSFER
    }

    private WristState wristState;
    private ClawState clawState;
    private IntakeState intakeState;
    private PitchState pitchState;
    private ExtendoState extendoState;
    private int extendoHeight = 0;


    private double WristHomePos = 0.569;
    private double WristDiagonalPosRight = 0.7;
    private double WristDiagonalPosLeft = 0.37;
    private double WristTransferPos = 0;

    private double[] ExtendoHeights = {
            -500,
            -1000,
            -1500,
            -2000,
            -2500,
            -3000,
            -3500
    };
    private double LIFT_MANUAL_FACTOR=10;
    private int HomeExtendoPos = -10;
    private int ExtendExtendoPos = 800;

    private double IntakePos = 96;
    private double IntakeTransferPos = 173;
    private double IntakeDepositPos = 340;

    private double PitchIntakePos = 0.24;
    private double PitchTransferPos = 0.85;

            //0.86
    private double PitchDepositPos = 0.4;

    private double OpenClawPos =0.48;
    private double CloseClawPos = 0.16;




    public IntakeSubsystem()
    {

    }
    @Override
    public void periodic() {
        obot.Extendo.periodic();
    }

    @Override
    public void read() {
        obot.Extendo.read();
    }

    @Override
    public void write() {
        obot.Extendo.write();
    }

    @Override
    public void reset() {

    }

    public void update(WristState state)
    {
        wristState=state;
        switch(wristState)
        {
            case HOME:
                obot.wrist.setPosition(WristHomePos);
                break;
            case DIAGONAL_LEFT:
                obot.wrist.setPosition(WristDiagonalPosLeft);
                break;
            case TRANSFER:
                obot.wrist.setPosition(WristTransferPos);
                break;
            case DIAGONAL_RIGHT:
                obot.wrist.setPosition(WristDiagonalPosRight);
                break;
        }
    }

    public void update(PitchState state)
    {
        pitchState = state;
        switch (pitchState){
            case INTAKE:
                obot.pitch_intake.setPosition(PitchIntakePos);
                break;
            case TRANSFER:
                obot.pitch_intake.setPosition(PitchTransferPos);
                break;
            case DEPOSIT:
                obot.pitch_intake.setPosition(PitchDepositPos);
                break;
        }
    }

    public void update(ExtendoState state)
    {
        extendoState=state;
        switch(extendoState)
        {
            case HOME:
                obot.Extendo.setMotionProfileTargetPosition(HomeExtendoPos);
                break;
            case EXTEND:
                obot.Extendo.setMotionProfileTargetPosition(ExtendExtendoPos);
                break;
        }
    }

    public void update(IntakeState state){
        intakeState=state;
        switch (intakeState){
            case INTAKE:
                obot.rightIntake.setPosition(IntakePos/360);
                obot.leftIntake.setPosition(IntakePos/360);
                break;
            case TRANSFER:
                obot.rightIntake.setPosition(IntakeTransferPos/360);
                obot.leftIntake.setPosition(IntakeTransferPos/360);
                break;
            case DEPOSIT:
                obot.rightIntake.setPosition(IntakeDepositPos/360);
                obot.leftIntake.setPosition(IntakeDepositPos/360);
                break;
        }
    }

    public void update(ClawState state){
        clawState = state;
        switch (clawState){
            case OPEN:
                obot.claw.setPosition(OpenClawPos);
                break;
            case CLOSE:
                obot.claw.setPosition(CloseClawPos);
                break;
        }
    }

    public int getExtendoHeightIndex() {
        return extendoHeight;
    }
    public double getLiftHeight() {
        return ExtendoHeights[getExtendoHeightIndex()];
    }


    public void incrementExtendoHeight(int amount) {
        this.extendoHeight = (int) MathUtil.clamp(getExtendoHeightIndex() + amount, 0, 15);
    }
    public void setExtendoHeight(int amount) {
        this.extendoHeight = (int) MathUtil.clamp(amount, 0, 15);
    }
}
