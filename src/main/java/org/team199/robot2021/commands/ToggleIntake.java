package org.team199.robot2021.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team199.robot2021.subsystems.Intake;

public class ToggleIntake extends CommandBase {

    private Intake intake;

    public ToggleIntake(Intake intake) {
        addRequirements(this.intake = intake);
    }

    public void initialize() {
        if (intake.isDeployed()) {
            intake.retract();
            intake.stop();
        } else {
            intake.deploy();
            intake.intake();
        }
    }

    public boolean isFinished() {
        return true;
    }

    public void end(boolean interrupted) {
    }
}
