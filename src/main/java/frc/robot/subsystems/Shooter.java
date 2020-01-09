package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.ShooterTargetSpeed;
import frc.robot.commands.EjectCell;

public class Shooter implements Subsystem {
    private double speed;
    public Shooter() {
        
        /*
            When button pressed (command already in wpilib)
        */

        ShooterTargetSpeed.ShooterTargetSpeed(speed);
        EjectCell.EjectCell();

    }



    
}