package org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.OuttakeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;

public class LiftPosCommand extends SequentialCommandGroup {
    public LiftPosCommand(Obot obot, LiftSubsystem.LiftState liftState){
        super(
                new InstantCommand(() -> obot.liftSubsystem.update(liftState))
        );
    }
}
