package org.team199.robot2021.commands;

import org.team199.robot2021.subsystems.Turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CalibrateTurret extends CommandBase {
    
    private final Turret turret;
    private double speed;
    private boolean cancel;
    private boolean hasTurned180;

    private final double alignError = 8;
    private final double alignSpeed = 0.25;

    public CalibrateTurret(Turret turret) {
        addRequirements(this.turret = turret);
    }

    @Override
    public void initialize() {
        cancel = false;
        hasTurned180 = false;
        speed = alignSpeed;
        if(turret.isAtLimit()) {
            cancel = true;
        }
    }

    @Override
    public void execute() {
        if(cancel) {
            return;
        }
        if(turret.isAtHome() && hasTurned180) {
            // System.out.println(speed + " " + SmartDashboard.getNumber("Simulated Turret Position", 0D));
            turret.setPosition(Math.copySign(alignError, -speed));
            // System.out.println(turret.getPosition() - SmartDashboard.getNumber("Simulated Turret Position", 0D));
            speed = 0;
        } else if(turret.limited(speed)) {
            speed *= -1;
            hasTurned180 = true;
        }
        turret.set(speed);
    }

    @Override
    public boolean isFinished() {
        if(cancel || (turret.isAtHome() && hasTurned180)) {
            System.out.println("fin");
        }
        return cancel || (turret.isAtHome() && hasTurned180);
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
    
}
