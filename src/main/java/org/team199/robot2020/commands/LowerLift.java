/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import org.team199.robot2020.subsystems.Climber;
//import motors and button
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class LowerLift extends CommandBase {
  //initialize motors
  Climber climber;
  double speed;
  public LowerLift(Climber climber, double speed) {
    this.climber = climber;
    this.speed = speed;
  }
  
  @Override
  public void initialize() {     
  }

  @Override
  public void execute() {
    //control the motor with a button on the remote
    //when the button is pressed the motor raises the lift and then slowly starts to lower
    climber.lowerLift(speed);
  }

  
  @Override
  public void end(final boolean interrupted) {
    //motors stop running
    climber.stopLift();
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
