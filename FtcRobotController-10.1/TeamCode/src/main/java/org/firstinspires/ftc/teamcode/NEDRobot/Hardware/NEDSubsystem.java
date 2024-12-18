package org.firstinspires.ftc.teamcode.NEDRobot.Hardware;

import com.arcrobotics.ftclib.command.SubsystemBase;

public abstract class NEDSubsystem extends SubsystemBase {

    public abstract void periodic();
    public abstract void read();
    public abstract void write();
    public abstract void reset();
}
