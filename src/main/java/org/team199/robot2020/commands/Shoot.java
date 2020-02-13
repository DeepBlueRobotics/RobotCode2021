package org.team199.robot2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team199.robot2020.subsystems.Feeder;

public class Shoot extends CommandBase {
    Feeder feeder;

    public Shoot(Feeder feeder) {
        this.feeder = feeder;
        addRequirements(feeder);
    }

    public void initialize() {
        feeder.eject();
    }

    public boolean isFinished() {
        return false; // wait until flywheel is up to speed 
    }

    public void end(boolean interrupted) {
        feeder.stop();
    }
}
