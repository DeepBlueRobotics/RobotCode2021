package org.team199.robot2020.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team199.robot2020.subsystems.Intake;
import org.team199.robot2020.commands.DeployIntake;
import org.team199.robot2020.commands.RunIntakeAndHopper;
public class DeployAndRunIntake extends SequentialCommandGroup{
    
    public DeployAndRunIntake(Intake intake){
        
        addCommands(new DeployIntake(intake), new RunIntakeAndHopper(intake));
    }
}