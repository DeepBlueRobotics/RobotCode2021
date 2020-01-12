package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.lang.module.ModuleDescriptor.Requires;
import frc.robot.subsystems.Intake;

public class RunIntakeAndHopper extends CommandBase {
    Intake intake;

    public RunIntakeAndHopper(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    public void initialize(){
    }

    public void execute(){
        //run the motors
    }

    public boolean isFinished(){
        return false;
        //return true when the button is pressed
    }

    public void end(boolean interrupted){
        //stop motors
    }
}
