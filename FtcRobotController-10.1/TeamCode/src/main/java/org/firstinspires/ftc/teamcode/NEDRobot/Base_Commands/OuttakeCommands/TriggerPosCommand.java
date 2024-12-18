package org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.OuttakeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;

public class TriggerPosCommand extends SequentialCommandGroup {

    public TriggerPosCommand(Obot obot, LiftSubsystem.TriggerState triggerState){
        super(
            new InstantCommand(() -> obot.liftSubsystem.update(triggerState))
        );

    }
}
