package org.team199.robot2021.commands;

import org.team199.robot2021.subsystems.Turret;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.Limelight;

public class ShooterHorizontalAim extends CommandBase {
    private final Limelight limelight;
    private final Turret turret;
    private final double txRange = 2.0;     // TODO: Determine correct txRange
    private int state;
    private double rot180Speed;
    private static final int REVERSE_ZERO_TV_MASK = 1 << 0;
    private static final int ROTATE_180_MASK = 1 << 1;
    public ShooterHorizontalAim(Turret turret, Limelight limelight){
        this.turret = turret;
        this.limelight = limelight;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        state = 0;
    }

    public void execute() {
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
        if((state & ROTATE_180_MASK) == ROTATE_180_MASK) {
            turret.set(rot180Speed);
            if(turret.limited(rot180Speed)) {
                state = state & ~ROTATE_180_MASK;
            }
        } else {
            double adjustment = limelight.steeringAssist(turret.getPosition());
            if(tv == 1.0) {
                state = state & ~REVERSE_ZERO_TV_MASK;
                if(turret.limited(adjustment)) {
                    state = state | ROTATE_180_MASK;
                    rot180Speed = Math.copySign(1.0, adjustment) * -1.0;
                }
            } else {
                if((state & REVERSE_ZERO_TV_MASK) == REVERSE_ZERO_TV_MASK) {
                    adjustment *= -1;
                }
                if(turret.limited(adjustment)) {
                    state = state ^ REVERSE_ZERO_TV_MASK;
                }
            }
            turret.set(adjustment);
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.set(0);
    }

    public boolean isFinished() {
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
        return (Math.abs(tx) < txRange) && tv == 1.0;
    }
}