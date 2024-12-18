package org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.OuttakeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;

public class BucketPosCommand extends SequentialCommandGroup {

    public BucketPosCommand(Obot obot, LiftSubsystem.BucketState bucketState){
        super(
                new InstantCommand(() -> obot.liftSubsystem.update(bucketState))
        );
    }
}
