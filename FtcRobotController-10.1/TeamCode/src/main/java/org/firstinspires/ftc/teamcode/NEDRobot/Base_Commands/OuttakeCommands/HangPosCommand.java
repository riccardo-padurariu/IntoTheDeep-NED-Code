package org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.OuttakeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;

public class HangPosCommand extends SequentialCommandGroup {
    public HangPosCommand(Obot obot, LiftSubsystem.HangState hangState){
        super(
                new InstantCommand(() -> obot.liftSubsystem.update(hangState))
        );
    }
}