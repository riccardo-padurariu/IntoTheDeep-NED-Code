package org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.IntakeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;

public class ExtendoPosCommand extends SequentialCommandGroup {
    public ExtendoPosCommand(Obot obot, IntakeSubsystem.ExtendoState extendoState){
        super(
                new InstantCommand(() -> obot.intakeSubsystem.update(extendoState))
        );
    }
}
