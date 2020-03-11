/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020.commands;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ToggleCamera extends InstantCommand {

  private boolean state;
  /**
   * Toggles between the two cameras.
   */
  public ToggleCamera() {
    super();
    state = false;
  }

  // Called once when the command executes
  @Override
  public void initialize() {
    if (state) {
      setCamera(2);
    } else {
      setCamera(1);
    }
    state = !state;
  }

  public static void setCamera(int camera) {
    camera = camera < 0 ? 0 : camera > 2 ? 2 : camera;
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(camera);
  }

}