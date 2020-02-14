package org.team199.robot2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team199.lib.Limelight;
import org.team199.robot2020.subsystems.Drivetrain;
public class ShooterHorizontalAim extends CommandBase {
    private final Limelight limelight;
    private final Drivetrain drivetrain;
    /*
    get horizonal adjustment by rotating drivetrain from limelight 
    tell drivetrain to rotate 
    repeat?
    */
    public ShooterHorizontalAim(Drivetrain drivetrain, Limelight limelight){
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements(drivetrain);
    }

    public boolean isFinished() {
        return true;
    }
}