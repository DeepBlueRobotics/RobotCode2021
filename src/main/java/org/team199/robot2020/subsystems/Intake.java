package org.team199.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.team199.lib.MotorControllerFactory;
import org.team199.robot2020.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final WPI_TalonSRX intakeMotor = MotorControllerFactory.createTalon(Constants.Intake.INTAKE_MOTOR);
    //TODO: declare and initialize pnematics system that moves intake up and down

    public Intake(){
    }

    public void intake()
    {
        intakeMotor.set(1);
        //TODO: put intake down if its not already
    }
    public void outtake()
    {
        intakeMotor.set(-1);
        //TODO: put intake up
    }
    public void stopIntake()
    {
        intakeMotor.set(0);
        //TODO: put intake down if its not already
    }
}