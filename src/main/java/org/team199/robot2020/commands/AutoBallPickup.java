package org.team199.robot2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team199.robot2020.subsystems.Feeder;
import org.team199.robot2020.subsystems.Intake;

public class AutoBallPickup extends CommandBase {
    private final Feeder feeder;
    private final Intake intake;

    public AutoBallPickup(Feeder feeder, Intake intake) {
        this.feeder = feeder;
        this.intake = intake;
        addRequirements(feeder, intake);
    }

    public void initialize() {
    }

    public void execute() {
    }

    public boolean isFinished() {
        return true;
    }

    public void end(boolean interrupted) {
    }
}