package org.team199.robot2020.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team199.robot2020.subsystems.Intake;

public class RunIntakeAndFeeder extends CommandBase {
    Intake intake;

    public RunIntakeAndFeeder(Intake intake){
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
