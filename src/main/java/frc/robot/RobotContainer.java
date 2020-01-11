/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Drivetrain;

/**
 * Add your docs here.
 */
public class RobotContainer {
    Drivetrain dt;
    Joystick lJoy;
    Joystick rJoy;
    TeleopDrive teleopDrive;


    public RobotContainer(Drivetrain dt,Joystick lJoy,Joystick rJoy){
        configureButtonBindings();
        this.dt=dt;
        this.lJoy=lJoy;
        this.rJoy=rJoy;
        teleopDrive=new TeleopDrive(dt,lJoy,rJoy);
    }

    private void configureButtonBindings(){
    }

    public CommandBase getTeleopCommand(){
        return teleopDrive;
    }




}








    

