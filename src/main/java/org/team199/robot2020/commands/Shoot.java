package org.team199.robot2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.team199.robot2020.subsystems.Shooter;
import org.team199.robot2020.commands.EjectCell;

public class Shoot extends CommandBase {
    Shooter shooter;

    public Shoot(Shooter shooter) {
        this.shooter = shooter;
    }

    public void initialize() {
        shooter.setTargetSpeed(Shooter.SHOOTING_SPEED);
    }

    public boolean isFinished() {
        return false; // wait until flywheel is up to speed 
    }

    public void end() {
        CommandScheduler.getInstance().schedule(new EjectCell(shooter));
    }
}
