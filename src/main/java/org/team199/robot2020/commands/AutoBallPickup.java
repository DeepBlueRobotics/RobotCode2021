package org.team199.robot2020.commands;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team199.robot2020.subsystems.Drivetrain;
import org.team199.robot2020.subsystems.Feeder;
import org.team199.robot2020.subsystems.Intake;

public class AutoBallPickup extends CommandBase {
    private final Feeder feeder;
    private final Intake intake;
    private final Drivetrain drivetrain;
    private final PowerDistributionPanel pdp;
    private final int feederPDPPort, numBallsRequired;
    private int numBallsEntered = 0;
    private boolean isIntaking = false;
    private final double currentThreshold = 100.0;      // TODO: Determine correct current threshold

    public AutoBallPickup(Feeder feeder, Intake intake, Drivetrain drivetrain, PowerDistributionPanel pdp, int feederPDPPort, int numBallsRequired) {
        this.feeder = feeder;
        this.intake = intake;
        this.drivetrain = drivetrain;
        this.pdp = pdp;
        this.feederPDPPort = feederPDPPort;
        this.numBallsRequired = numBallsRequired;
        addRequirements(feeder, intake);
    }

    public void initialize() {
    }

    public void execute() {
        // Unless the ball has jammed, run intake and feeder normally.
        if (!isJam()) {
            intake.intake();
            if (feeder.isCellEntering()) {
                // Only run the feeder if there is no ball at the shooter.
                if (!feeder.isCellAtShooter()) {
                    isIntaking = true;
                    feeder.runForward();
                    drivetrain.tankDrive(0.0, 0.0, false);
                } else {
                    feeder.stop();
                    if(isIntaking) {
                        isIntaking = false;
                        numBallsEntered++;
                    }
                }
            } else {
                if(isIntaking) {
                    isIntaking = false;
                    numBallsEntered++;
                }
            }
        } else {
            // Stop the intake and regurgitate to stop the jam.
            intake.stop();
            feeder.runBackward();
            // Hopefully, the path code should be running and setting drivetrain values and this code constantly overrides it and sets it to zero.
            drivetrain.tankDrive(0.0, 0.0, false);
        }
    }

    public boolean isFinished() {
        // Command should only end when it is interrupted.
        return numBallsEntered >= numBallsRequired;
    }

    public void end(boolean interrupted) {
        feeder.stop();
        intake.stop();
    }

    public boolean isJam() {
        // Ball jams if there is too much current draw.
        return pdp.getCurrent(feederPDPPort) > currentThreshold;
    }
}