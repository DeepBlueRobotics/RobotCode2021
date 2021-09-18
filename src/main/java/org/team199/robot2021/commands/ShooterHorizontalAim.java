package org.team199.robot2021.commands;

import org.team199.robot2021.subsystems.Turret;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.Limelight;

public class ShooterHorizontalAim extends CommandBase {
    private final Limelight limelight;
    private final Turret turret;
    private final double txRange = 2.0;     // TODO: Determine correct txRange
    private boolean reverseZeroTv;
    private boolean rotate180;
    private double rot180Speed;
    public ShooterHorizontalAim(Turret turret, Limelight limelight){
        this.turret = turret;
        this.limelight = limelight;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        reverseZeroTv = false;
        rotate180 = false;
    }

    public void execute() {
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
        if(rotate180) {
            turret.set(rot180Speed);
            if(turret.limited(rot180Speed)) {
                rotate180 = false;
            }
        } else {
            double adjustment = limelight.steeringAssist(turret.getPosition());
            if(tv == 1.0) {
                reverseZeroTv = false;
                if(turret.limited(adjustment)) {
                    rotate180 = true;
                    rot180Speed = Math.copySign(1.0, adjustment) * -1.0;
                }
            } else {
                if(reverseZeroTv) {
                    adjustment *= -1;
                }
                if(turret.limited(adjustment)) {
                    reverseZeroTv = !reverseZeroTv;
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