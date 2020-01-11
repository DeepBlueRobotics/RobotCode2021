
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.Shoot;

public class OI {
    Joystick leftJoy, rightJoy, manipulator;
    JoystickButton shootBtn;

    OI(Shooter shooter) {

        leftJoy = new Joystick(0);
        rightJoy = new Joystick(1);
        manipulator = new Joystick(2);
        shootBtn = new JoystickButton(manipulator, 1); //TODO: Set joystick button number
        shootBtn.whenPressed(new Shoot(shooter));
    }
}