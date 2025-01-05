package org.firstinspires.ftc.teamcode.NEDRobot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.util.MathUtil;

public class LiftSubsystem extends NEDSubsystem {
    private final Obot obot = Obot.getInstance();

    public enum TriggerState{
        CLOSE,
        OPEN,
        CLOSE_DEPOSIT
    }

    public enum LiftState{
        HOME,
        LOW_BASKET,
        HIGH_BASKET
    }

    public enum BucketState{
        CLAMP,
        BASKET,
        TRANSFER,
        CLAMP_DEPOSIT,
        CLAMP_TRANSFER,
        DEPOSIT_AUTO
    }

    public enum HangState{
        HOME,
        SWITCH
    }

    private TriggerState triggerState;
    private BucketState bucketState;
    private LiftState liftState;
    private HangState hangState;
    private int liftHeight = 0;


    private double[] LiftHeights = {
            -500,
            -1000,
            -1500,
            -2000,
            -2500,
            -3000,
            -3500
    };
    private double LIFT_MANUAL_FACTOR=10;
    private int HomeLiftPos = 0;
    private int LowBasketLiftPos = 440;
    private int HighBasketLiftPos= 815;

    private double BucketTranferPos = 303;//340
    private double BucketBasketPos = 100;
    private double BucketClampPos = 0;
    private double BucketClampDepositPos = 265;
    private double BucketClampTransferPos = 330;

    private double OpenTriggerPos =1;
    private double CloseTriggerPos = 0.7;
    private double CloseDepositPos = 0.81;

    private double HangHomePos = 0;
    private double HangSwitchPos = 0;



    public LiftSubsystem()
    {

    }
    @Override
    public void periodic() {
        obot.Lift.periodic();
    }

    @Override
    public void read() {
        obot.Lift.read();
    }

    @Override
    public void write() {
        obot.Lift.write();
    }

    @Override
    public void reset() {

    }

    public void update(BucketState state)
    {
        bucketState = state;
        switch (bucketState)
        {
            case CLAMP:
                obot.leftBucket.setPosition(BucketClampPos/360);
                obot.rightBucket.setPosition(BucketClampPos/360);
                break;
            case BASKET:
                obot.leftBucket.setPosition(BucketBasketPos/360);
                obot.rightBucket.setPosition(BucketBasketPos/360);
                break;
            case TRANSFER:
                obot.leftBucket.setPosition(BucketTranferPos/360);
                obot.rightBucket.setPosition(BucketTranferPos/360);
                break;
            case CLAMP_DEPOSIT:
                obot.leftBucket.setPosition(BucketClampDepositPos/360);
                obot.rightBucket.setPosition(BucketClampDepositPos/360);
                break;
            case CLAMP_TRANSFER:
                obot.leftBucket.setPosition(BucketClampTransferPos/360);
                obot.rightBucket.setPosition(BucketClampTransferPos/360);
                break;
        }
    }

    public void update(LiftState state)
    {
        liftState=state;
        switch(liftState)
        {
            case HOME:
                obot.Lift.setMotionProfileTargetPosition(HomeLiftPos);
                break;
            case LOW_BASKET:
                obot.Lift.setMotionProfileTargetPosition(LowBasketLiftPos);
                break;
            case HIGH_BASKET:
                obot.Lift.setMotionProfileTargetPosition(HighBasketLiftPos);
                break;
        }
    }

    public void update(HangState state){
        hangState=state;
        switch (hangState){
            case HOME:
                obot.hang.setPosition(HangHomePos);
                break;
            case SWITCH:
                obot.hang.setPosition(HangSwitchPos);
        }
    }

    public void update(TriggerState state){
        triggerState = state;
        switch (triggerState){
            case OPEN:
                obot.trigger.setPosition(OpenTriggerPos);
                break;
            case CLOSE:
                obot.trigger.setPosition(CloseTriggerPos);
                break;
            case CLOSE_DEPOSIT:
                obot.trigger.setPosition(CloseDepositPos);
        }
    }

    public int getLiftHeightIndex() {
        return liftHeight;
    }
    public double getLiftHeight() {
        return LiftHeights[getLiftHeightIndex()];
    }


    public void incrementLiftHeight(int amount) {
        this.liftHeight = (int) MathUtil.clamp(getLiftHeightIndex() + amount, 0, 15);
    }
    public void setLiftHeight(int amount) {
        this.liftHeight = (int) MathUtil.clamp(amount, 0, 15);
    }

}
