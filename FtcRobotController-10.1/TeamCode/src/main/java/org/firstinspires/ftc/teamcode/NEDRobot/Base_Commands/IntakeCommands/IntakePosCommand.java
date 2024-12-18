package org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.IntakeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;

public class IntakePosCommand extends SequentialCommandGroup {

    public IntakePosCommand(Obot obot, IntakeSubsystem.IntakeState intakeState){
        super(
                new InstantCommand(() -> obot.intakeSubsystem.update(intakeState))
        );
    }
}
