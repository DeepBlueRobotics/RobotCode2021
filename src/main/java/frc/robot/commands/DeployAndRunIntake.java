package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.RunIntakeAndHopper;
public class DeployAndRunIntake extends SequentialCommandGroup{
    
    public DeployAndRunIntake(Intake intake){
        
        addCommands(new DeployIntake(intake), new RunIntakeAndHopper(intake));
    }
}