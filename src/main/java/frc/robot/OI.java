/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

    Joystick leftJoy, rightJoy, manipulator;
    JoystickButton arcadeOrTankBtn;

    OI(Drivetrain dt){

        leftJoy = new Joystick(0);
        rightJoy = new Joystick(1);
        manipulator = new Joystick(2);


        arcadeOrTankBtn = new JoystickButton(leftJoy, 4);
        arcadeOrTankBtn.whenPressed(new InstantCommand(() -> 
            SmartDashboard.putBoolean("Arcade Drive", !SmartDashboard.getBoolean("Arcade Drive",false))));


    }






}