package org.team199.robot2021.commands;

import org.team199.robot2021.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CalibrateTurret extends CommandBase {
    
    private final Turret turret;
    private double speed;
    private double initialPosition;
    private double panSetpoint;
    private int state;

    private static final int PAN_LIMIT_MASK = 1 << 0;
    private static final int APROX_ALIGN_MASK = 1 << 1;
    private static final int FINAL_ALIGN_MASK = 1 << 2;
    private static final int SIGN_MASK = 1 << 4;

    public CalibrateTurret(Turret turret) {
        addRequirements(this.turret = turret);
    }

    @Override
    public void initialize() {
        state = 0;
        initialPosition = turret.getPosition();
        panSetpoint = 10;
        speed = 0.25;
    }

    @Override
    public void execute() {
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
                turret.setPosition((state & SIGN_MASK) == SIGN_MASK ? -170 : 170);
                speed *= -1;
                state = state | APROX_ALIGN_MASK;
            } else if(Math.abs(turret.getPosition() - initialPosition) >= panSetpoint) {
                speed *= -1;
                if((state | PAN_LIMIT_MASK) == PAN_LIMIT_MASK) {
                    panSetpoint += 10;
                }
                state = state ^ (PAN_LIMIT_MASK | SIGN_MASK);
            }
        }
        turret.set(speed);
    }

    @Override
    public boolean isFinished() {
        return (turret.isAtHome() && (state & FINAL_ALIGN_MASK) == FINAL_ALIGN_MASK);
    }

    @Override
    public void end(boolean interrupted) {
        if(!interrupted) {
            turret.stop();
            turret.setPosition(0);
        }
    }
    
}
