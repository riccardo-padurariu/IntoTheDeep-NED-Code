package org.firstinspires.ftc.teamcode.NEDRobot.Subsystems;

import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.util.MathUtil;

public class LiftSubsystem extends NEDSubsystem {
    private final Obot obot = Obot.getInstance();

    public enum TriggerState{
        CLOSE,
        OPEN
    }

    public enum LiftState{
        HOME,
        LOW_BASKET,
        HIGH_BASKET
    }

    public enum BucketState{
        CLAMP,
        BASKET,
        TRANSFER
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
    private int LowBasketLiftPos = 350;
    private int HighBasketLiftPos= 650;

    private double BucketTranferPos = 303;
    private double BucketBasketPos = 120;
    private double BucketClampPos = 0;

    private double OpenTriggerPos =0.5;
    private double CloseTriggerPos = 0.67;

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