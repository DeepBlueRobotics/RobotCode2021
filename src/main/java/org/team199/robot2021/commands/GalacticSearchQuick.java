// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2021.commands;

import java.util.Arrays;

import org.team199.robot2021.subsystems.Drivetrain;
import org.team199.robot2021.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GalacticSearchQuick extends SequentialCommandGroup {
  /** Creates a new GalacticSearchQuick. */
  public final double start_Y = 87; // 87 inches off of ground
  
  public final double red_a_dist = Math.hypot(60, Math.abs(90-start_Y));
  public final double red_b_dist = Math.hypot(60, Math.abs(120-start_Y));
  public final double blue_a_dist = Math.hypot(150, Math.abs(30-start_Y));
  public final double blue_b_dist = Math.hypot(150, Math.abs(60-start_Y));

  public GalacticSearchQuick(Drivetrain dt, Intake intake, Limelight lime, double cameraHeight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(dt, intake);
    
    // TODO: Actually get the data from the Limelight
    // This is temporary pseudocode
    double realDist = 1; // This is a temporary value because I haven't done the limelight stuff yet

    int pathInd = minDist(realDist, red_a_dist, red_b_dist, blue_a_dist, blue_b_dist);

    // Run paths based on which it finds
    // TODO: Add Paths for start_Y =87

    addCommands();
  }

  public int minDist(double real, double redA, double redB, double blueA, double blueB) {
    double d_ra = Math.abs(redA-real);
    double d_rb = Math.abs(redB-real);
    double d_ba = Math.abs(blueA-real);
    double d_bb = Math.abs(blueB-real);

    int minInd = 0;

    double[] distances = new double[] {d_ra,d_rb,d_ba,d_bb};

    for (int i = 1; i < distances.length; i++) {
      if (distances[i] < distances[minInd]) minInd = i;
    }

    return minInd;
  }
}
