package org.team199.robot2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team199.robot2020.subsystems.Feeder;
import org.team199.robot2020.subsystems.Intake;

public class Shoot extends CommandBase {
    private final Feeder feeder;
    private final Intake intake;
    private final double limitDistance = 10000;    // TODO: Figure out the correct value.

    public Shoot(Feeder feeder, Intake intake) {
        this.feeder = feeder;
        this.intake = intake;
        addRequirements(feeder);
    }

    public void initialize() {
        feeder.eject();
    }

    public void execute() {
        if (feeder.getCellPosition() > limitDistance) intake.outtake();
        else intake.stop();
    }

    public boolean isFinished() {
        return false; // wait until flywheel is up to speed 
    }

    public void end(boolean interrupted) {
        feeder.stop();
        feeder.reset();
    }
}
