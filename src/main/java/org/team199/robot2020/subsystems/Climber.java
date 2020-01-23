package org.team199.robot2020.subsystems;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;

import org.team199.lib.MotorControllerFactory;
import org.team199.robot2020.Constants;

public class Climber {
    public double elevatorLength;
    private final CANSparkMax liftRobotMotor = MotorControllerFactory.createSparkMax(Constants.Drive.liftRobotMotor);
    private final VictorSPX liftMotor;
    public Climber(){
        
    }

    public void Init(){
        elevatorLength = 0;
    }

    public void Periodic() {

    }
}