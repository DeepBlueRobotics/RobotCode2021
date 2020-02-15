package org.team199.robot2020.commands;

import org.team199.lib.RobotPath;
import org.team199.robot2020.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// TODO: Change command group type to correct group type. I don't think SequentialCommandGroup works in this case.
public class AutoShootAndDrive extends SequentialCommandGroup {
    // TODO: Add shooter to subsytem requirements.
    public AutoShootAndDrive(Intake in, RobotPath path) {
        addCommands(
            // TODO: Put the shooter command here.
            // TODO: Put the "run intake" command here.
            path.getPathCommand()
            // TODO: Put the "stop intake" command here.
            // TODO: Put the shooter command here.
        );
    }
}