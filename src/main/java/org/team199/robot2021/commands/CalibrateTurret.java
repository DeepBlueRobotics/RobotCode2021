package org.team199.robot2021.commands;

import org.team199.robot2021.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CalibrateTurret extends CommandBase {
    
    private final Turret turret;
    private double speed;
    private int state;
    private boolean cancel;

    private static final int APROX_ALIGN_MASK = 1 << 0;
    private static final int FINAL_ALIGN_MASK = 1 << 1;

    public CalibrateTurret(Turret turret) {
        addRequirements(this.turret = turret);
    }

    @Override
    public void initialize() {
        cancel = false;
        state = 0;
        speed = 0.25;
        if(turret.isAtLimit()) {
            cancel = true;
        }
    }

    @Override
    public void execute() {
        if(cancel) {
            return;
        }
        if((state & FINAL_ALIGN_MASK) == FINAL_ALIGN_MASK) {
            if(Math.abs(turret.getPosition()) >= 10 && Math.signum(turret.getPosition()) == Math.signum(speed)) {
                speed *= -1;
            }
        } else if((state & APROX_ALIGN_MASK) == APROX_ALIGN_MASK) {
            if(Math.signum(turret.getPosition()) == Math.signum(speed)) {
                turret.setPosition(0);
                speed = Math.copySign(0.01, speed);
                state = state | FINAL_ALIGN_MASK;
            }
        } else {
            if(turret.isAtHome()) {
                turret.setPosition(0);
                speed = 0.01;
                state = state | FINAL_ALIGN_MASK;
            } else if(turret.limited(speed)) {
                turret.setPosition(180);
                speed *= -1;
                state = state | APROX_ALIGN_MASK;
            }
        }
        turret.set(speed);
    }

    @Override
    public boolean isFinished() {
        return cancel || (turret.isAtHome() && (state & FINAL_ALIGN_MASK) == FINAL_ALIGN_MASK);
    }

    @Override
    public void end(boolean interrupted) {
        if(!interrupted) {
            turret.stop();
            turret.setPosition(0);
        }
    }
    
}
