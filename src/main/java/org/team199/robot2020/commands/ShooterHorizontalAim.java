package org.team199.robot2020.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team199.lib.Limelight;
import org.team199.robot2020.subsystems.Drivetrain;

public class ShooterHorizontalAim extends CommandBase {
    private final Limelight limelight;
    private final Drivetrain drivetrain;
    private final double txRange = 1.0;
    public ShooterHorizontalAim(Drivetrain drivetrain, Limelight limelight){
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements(drivetrain);
    }

    public void initialize() {
        limelight.stopSteer = false;
        limelight.setLight(true);
    }

    public void execute() {
        double adjustment = limelight.steeringAssist();
        drivetrain.tankDrive(adjustment, -adjustment, false);
    }

    public boolean isFinished() {
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
        return (Math.abs(tx) < txRange && Math.abs(SmartDashboard.getNumber("Prev_tx", 0)) < txRange) && tv == 1.0;
    }

    public void end(boolean interrupted) {
        limelight.setLight(false);
    }
}