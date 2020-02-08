/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2020;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import org.team199.lib.RobotPath;
import org.team199.robot2020.commands.TeleopDrive;
import org.team199.robot2020.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final DigitalInput autoSwitch1 = new DigitalInput(Constants.Drivetrain.AUTO_PATH_SWITCH_1_PORT);
    private final DigitalInput autoSwitch2 = new DigitalInput(Constants.Drivetrain.AUTO_PATH_SWITCH_2_PORT);
    private final Drivetrain drivetrain = new Drivetrain();
    private final Joystick leftJoy = new Joystick(Constants.OI.LeftJoy.PORT);
    private final Joystick rightJoy = new Joystick(Constants.OI.RightJoy.PORT);
    private RobotPath[] paths;

    public RobotContainer() {
        configureButtonBindings();
        drivetrain.setDefaultCommand(new TeleopDrive(drivetrain, leftJoy, rightJoy));
        paths = new RobotPath[6];
        loadPath(Path.BLUE1, "Blue1");
        loadPath(Path.BLUE2, "Blue2");
        loadPath(Path.BLUE3, "Blue3");
        loadPath(Path.RED1, "Red1");
        loadPath(Path.RED2, "Red2");
        loadPath(Path.RED3, "Red3");
    }

    private void configureButtonBindings() {
        // Arcade/Tank drive button
        new JoystickButton(leftJoy, Constants.OI.LeftJoy.ARCADETANK_DRIVE_BUTTON)
                .whenPressed(new InstantCommand(() -> SmartDashboard.putBoolean("Arcade Drive",
                !SmartDashboard.getBoolean("Arcade Drive", false))));

        // characterize drive button
        new JoystickButton(leftJoy, Constants.OI.LeftJoy.CHARACTERIZED_DRIVE_BUTTON)
                .whenPressed(new InstantCommand(() -> SmartDashboard.putBoolean("Characterized Drive",
                        !SmartDashboard.getBoolean("Characterized Drive", false))));
    }

    public Command getAutonomousCommand() {
        try {
            RobotPath path = paths[getPath().idx];
            if(path == null) {
                throw new Exception();
            }
            return path.getPathCommand();
        } catch(Exception e) {
            return new InstantCommand();
        }
    }

    public Path getPath() {
        Path outPath = Path.OFF;
        if(autoSwitch1.get()) {
            if(autoSwitch2.get()) {
                outPath = Path.BLUE3;
            } else {
                outPath = Path.BLUE1;
            }
        } else if(autoSwitch2.get()) {
            outPath = Path.BLUE2;
        } else {
            outPath = Path.OFF;
        }
        return outPath.toSide(DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue);
    }

    private void loadPath(Path path, String pathName) {
        try {
            paths[path.idx] = new RobotPath(pathName);
        } catch(Exception e) {
            System.err.println("Error Occured Loading Path: [" + path.name() + "," + pathName + "]");
            e.printStackTrace(System.err);
        }
    }public static enum Path {
        BLUE1(0), BLUE2(1), BLUE3(2), RED1(3), RED2(4), RED3(5), OFF(-1);

        public final int idx;

        private Path(int idx) {
            this.idx = idx;
        }

        public Path toSide(boolean isBlue) {
            switch(this) {
                case BLUE1:
                case RED1:
                    if(isBlue) {
                        return BLUE1;
                    }
                    return RED1;
                case BLUE2:
                case RED2:
                    if(isBlue) {
                        return BLUE2;
                    }
                    return RED2;
                case BLUE3:
                case RED3:
                    if(isBlue) {
                        return BLUE3;
                    }
                    return RED3;
                case OFF:
                default:
                    return OFF;
            }
        }
    }
}
