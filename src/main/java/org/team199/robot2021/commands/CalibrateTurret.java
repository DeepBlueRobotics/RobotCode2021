package org.team199.robot2021.commands;

import org.team199.robot2021.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CalibrateTurret extends CommandBase {
    
    private final Turret turret;
    private double speed;

    public CalibrateTurret(Turret turret) {
        addRequirements(this.turret = turret);
        speed = 0.01;
    }

    @Override
    public void execute() {
        if(turret.limited(speed)) {
            speed *= -1;
        }
        turret.set(speed);
    }

    @Override
    public boolean isFinished() {
        return turret.isAtHome();
    }

    @Override
    public void end(boolean interrupted) {
        if(!interrupted) {
            turret.stop();
            turret.resetPosition();
        }
    }
    
}
