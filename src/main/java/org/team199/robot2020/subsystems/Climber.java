package org.team199.robot2020.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Encoder;

import org.team199.lib.MotorControllerFactory;
import org.team199.robot2020.Constants;

public class Climber {
    public double elevatorLength;
    private final CANSparkMax liftRobotMotor = MotorControllerFactory.createSparkMax(Constants.Climber.liftRobotMotor);
    private final VictorSPX liftMotor = MotorControllerFactory.createVictor(Constants.Climber.liftMotor);
    private final Encoder liftEncoder = new Encoder();
    private final Encoder robtoEncoder = new Encoder();
    public Climber(){
        
    }

    public void raiseLift(){
        liftMotor.set(ControlMode.PercentOutput, 0.9);
    }

    public void lowerLift(double speed){
        liftMotor.set(ControlMode.PercentOutput, speed);
    }

    public void stopLift(){
        liftMotor.set(ControlMode.PercentOutput, 0);
    }

    public void Init(){
        elevatorLength = 0;
    }

    public void Periodic() {

    }
}