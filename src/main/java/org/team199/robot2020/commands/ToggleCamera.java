/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020.commands;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ToggleCamera extends InstantCommand {
  boolean toggle = false;

  UsbCamera camera1, camera2;
  VideoSink cameraServer;

  /**
   * Toggles between the two cameras.
   */
  public ToggleCamera(UsbCamera camera1, UsbCamera camera2, VideoSink cameraServer) {
    super();
    this.camera1 = camera1;
    this.camera2 = camera2;
    this.cameraServer = cameraServer;
  }

  // Called once when the command executes
  @Override
  public void initialize() {
    if (toggle) {
      cameraServer.setSource(camera1);
    } else {
      cameraServer.setSource(camera2);
    }
    toggle = !toggle;
  }

}