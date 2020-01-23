/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team199.robot2020.Constants;
import org.team199.lib.MotorControllerFactory;
import org.team199.robot2020.subsystems.Climber;
//import motors and encoder(s)
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Encoder;

public class RaiseRobot extends CommandBase {
 //initialize motors and encoder(s)
  public RaiseRobot(CANSparkMax liftRobotMotor) {
   //when a button is pressed(not held), then the motor starts to spin
   //which would gather the string and pull the robot upwards
   //encoders tell when the motor should stop based on the height of the robot relative to the bar 
  }

  
  @Override
  public void initialize() {
    
  }

  
  @Override
  public void execute() {
    //motors start to spin
    
  }

  
  @Override
  public void end(boolean interrupted) {
    //when the encoders reach a certain value, then the motors stop
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
