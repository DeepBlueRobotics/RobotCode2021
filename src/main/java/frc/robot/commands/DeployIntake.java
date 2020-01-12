package frc.robot.commands;
import java.lang.module.ModuleDescriptor.Requires;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class DeployIntake extends InstantCommand {
    Intake intake;
    public DeployIntake(Intake intake){
        this.intake = intake;
        addRequirements(intake);

    }

    public void initialize(){
        // make arm go down or up depending on the position of the arm
    }
}
