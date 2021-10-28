package org.team199.robot2021.commands;

import java.util.stream.DoubleStream;

import org.team199.robot2021.subsystems.Drivetrain;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.Limelight;

public class ShooterHorizontalAim extends CommandBase {
    private final Limelight limelight;
    private final Drivetrain drivetrain;
    private final double txRange = 2.0;     // TODO: Determine correct txRange
    public ShooterHorizontalAim(Drivetrain drivetrain, Limelight limelight){
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements(drivetrain);
    }

    public void execute() {
        double adjustment = limelight.steeringAssist();
        //drivetrain.tankDrive(adjustment, -adjustment, false);
    }

    public boolean isFinished() {
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
        ChassisSpeeds drivegaming = drivetrain.getSpeeds();
        return (Math.abs(tx) < txRange) && tv == 1.0 && DoubleStream.of(drivegaming.vxMetersPerSecond, drivegaming.vyMetersPerSecond, drivegaming.omegaRadiansPerSecond).map(Math::abs).sum() <= Units.inchesToMeters(1);
    }
}