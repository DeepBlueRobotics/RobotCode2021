
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DeployAndRunIntake;
import frc.robot.subsystems.Intake;

public class OI {
    Joystick leftJoy, rightJoy, manipulator;
    JoystickButton intakeBtn;

    OI(Intake intake) {
        leftJoy = new Joystick(0);
        rightJoy = new Joystick(1);
        manipulator = new Joystick(2);
        intakeBtn = new JoystickButton(manipulator, 2); //TODO: Set joystick button number
        intakeBtn.whenPressed(new DeployAndRunIntake(intake)); 
    }
}