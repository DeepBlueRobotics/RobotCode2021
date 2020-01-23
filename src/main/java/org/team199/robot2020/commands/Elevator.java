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
import edu.wpi.first.wpilibj.XboxController.Button;

public class Elevator extends CommandBase {
  //initialize motors
  VictorSPX liftMotor;
  Button liftButton;
  public Elevator(VictorSPX liftMotor, Button liftButton) {
    //control the motor with a button on the remote
    //when the button is pressed the motor raises the lift and then slowly starts to lower
    this.liftMotor = liftMotor;
    this.liftButton = liftButton;
    if /*buttonA pressed(use encoders for how much the motor will spin)*/ {
      //raise the elevator arm
    } elif /*buttonB pressed and held*/ {
      //lower the arm until the buttonB is let go
    }
  }

  private enum State {
    RAISING, //Going up
    LOWERING, //Lowering to the Bar
    ONBAR, //Getting on the Bar
    LOWERED //Lowering to start position
  }

  
  @Override
  public void initialize() {     
  }

  @Override
  public void execute() {
    //make motor run
    liftMotor.set(ControlMode.PercentOutput, 1);
  }

  
  @Override
  public void end(final boolean interrupted) {
    //motors stop running
    liftMotor.set(ControlMode.PercentOutput, 0);
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
