package org.team199.robot2021.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.playingwithfusion.TimeOfFlight;

import org.team199.robot2021.subsystems.Feeder;
import org.team199.robot2021.subsystems.Shooter;

public class AutoShoot extends CommandBase {
    private final Feeder feeder;
    private final Shooter shooter;
    private final TimeOfFlight distanceSensor;
    private final double[] distanceRange = {117.0, 224.0};  // TODO: Determine the correct distance interval
    private final double numBallsToShoot;
    private int ballCount = 0;

    private boolean ballCounted = false;
    private boolean stopInit = false;

    public AutoShoot(Feeder feeder, Shooter shooter, TimeOfFlight distanceSensor, int numBallsToShoot) {
        this.feeder = feeder;
        this.shooter = shooter;
        this.distanceSensor = distanceSensor;
        this.numBallsToShoot = numBallsToShoot;
        addRequirements(feeder, shooter);
    }

    public void execute() {
        // If shooter is at or near target speed, run the feeder.
        if (shooter.isAtTargetSpeed() && !stopInit) {
            feeder.shoot();
            stopInit = true;
        }
        // Ball is above the distance sensor and is being fed into the shooter
        if ((distanceSensor.getRange() < distanceRange[0]) && (ballCounted == false)) {
            ballCounted = true;
        }
        // Ball is touching the wheel of the shooter and is about be shot
        if ((distanceSensor.getRange() > distanceRange[1]) && (ballCounted == true)) {
            ballCounted = false;
            ballCount++;
        }      
        SmartDashboard.putNumber("BallCount", ballCount);
    }

    public boolean isFinished() {
        return ballCount == numBallsToShoot;
    }

    public void end(boolean interrupted) {
        feeder.stop();
    }
}