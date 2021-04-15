package org.team199.robot2021.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.lib.MotorControllerFactory;

import org.team199.robot2021.Constants;
import org.team199.robot2021.commands.RaiseRobot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class Climber extends SubsystemBase {
    private static final double kLiftConversionFactor =6.0/40 * Math.PI * 3; //TODO: confirm numbers (inches)
    private static final double kWinchConversionFactor = 1.0/9 * Math.PI; //TODO: confirm numbers (inches)

    //TODO: find good values and then set to final
    
    public static double kLiftDeploySpeed = RaiseRobot.liftDeploySpeed; //TODO: set correct speed
    public static double kWinchDeploySpeed = 1; //TODO: set correct speed
    public static double kLiftLowerSpeed = 1; //TODO: set correct speed
    public static double kLiftKeepSpeed = 0.06; //TODO: set correct speed
    public static double kLiftRetractSpeed = -0.3; //TODO: set correct speed
    public static double kWinchRetractSpeedFirst = 0.6; //TODO: set correct speed
    public static double kWinchRetractSpeedSecond = 0.6; //TODO: set correct speed
    public static double kLiftAdjustSpeed = 0.2; //TODO: set correct speed
    public static double kWinchAdjustSpeed = 0.2; //TODO: set correct speed
    public static double kHighVoltage = 0.1; //TODO: find voltage
    public static double kLowVoltage = 0.2; //TODO: find voltage
    public static double kArmRetryDistance = 5; //TODO: find distance

    public static final double kLiftHeight = 87; // TODO: set correct speed
    public static final double kLiftLowerHeight = 0;  //TODO: set this one probably
    public static final double kWinchEndHeight = 60; // TODO: set correct speed
    public static final double kWinchMidHeight = 60; //TODO: SET THIS ONE SPECIFICALLY 
    public static final double kWinchStartHeight = -80; // TODO: set correct speed
    private final WPI_VictorSPX liftMotor = MotorControllerFactory.createVictor(Constants.Drive.kClimberLift); //TODO SparkMax Become 775
    private final CANSparkMax winchMotor = MotorControllerFactory.createSparkMax(Constants.Drive.kClimberWinch); //TODO SparkMax Become Neo
    private final Encoder liftEnc = new Encoder(2, 3);
    private final CANEncoder winchEnc = winchMotor.getEncoder();
    private final PowerDistributionPanel powerDistributionPanel = new PowerDistributionPanel(1); //TODO: set correct module
    private final Joystick placeholder = new Joystick(1);
    private final Joystick placeholder2 = new Joystick(2);
    private final JoystickButton buttonCheck = new JoystickButton(placeholder, 1);
    private final JoystickButton checkHookAttached = new JoystickButton(placeholder2, 2);

    

    public Climber(){
        liftEnc.setDistancePerPulse(kLiftConversionFactor);
        winchEnc.setPositionConversionFactor(kWinchConversionFactor);
        winchEnc.setVelocityConversionFactor(kWinchConversionFactor / 60);
        liftEnc.reset();
        winchEnc.setPosition(kWinchStartHeight);
        winchMotor.setIdleMode(IdleMode.kCoast);
       // liftMotor.setSmartCurrentLimit(30);

        SmartDashboard.putNumber("Climber.kLiftDeploySpeed", kLiftDeploySpeed);
        SmartDashboard.putNumber("Climber.kWinchDeploySpeed", kWinchDeploySpeed);
        SmartDashboard.putNumber("Climber.kLiftKeepSpeed", kLiftKeepSpeed);
        SmartDashboard.putNumber("Climber.kLiftRetractSpeed", kLiftRetractSpeed);
        SmartDashboard.putNumber("Climber.kWinchRetractSpeedFirst", kWinchRetractSpeedFirst);
        SmartDashboard.putNumber("Climber.kLiftAdjustSpeed", kLiftAdjustSpeed);
    }

    public void periodic()  {
        kLiftDeploySpeed = SmartDashboard.getNumber("Climber.kLiftDeploySpeed", kLiftDeploySpeed);
        kWinchDeploySpeed = SmartDashboard.getNumber("Climber.kWinchDeploySpeed", kWinchDeploySpeed);
        kLiftKeepSpeed = SmartDashboard.getNumber("Climber.kLiftKeepSpeed", kLiftKeepSpeed);
        kLiftRetractSpeed = SmartDashboard.getNumber("Climber.kLiftRetractSpeed", kLiftRetractSpeed);
        kWinchRetractSpeedFirst = SmartDashboard.getNumber("Climber.kWinchRetractSpeedFirst", kWinchRetractSpeedFirst);
        kLiftAdjustSpeed = SmartDashboard.getNumber("Climber.kLiftAdjustSpeed", kLiftAdjustSpeed);
        SmartDashboard.putNumber("Lift Position", getLiftHeight());
        SmartDashboard.putNumber("Winch Position", getWinchHeight());
        SmartDashboard.putNumber("Lift Speed", getLiftSpeed());
        SmartDashboard.putNumber("Winch Speed", getWinchSpeed());
    }

    // OLD CHECK THAT STILL WORKS/REWRITE
    public void runLift(double speed) {
        System.out.println("CARRY ME BOII IS SPEED " + liftEnc.getRate());
        liftMotor.set(speed);
        if (liftEnc.getRate() == 0){
            System.out.println("UNWIND WHEN DONE TESTING!!!");
            System.out.println("UNWIND WHEN DONE TESTING!!!");
            System.out.println("UNWIND WHEN DONE TESTING!!!");
            System.out.println("UNWIND WHEN DONE TESTING!!!");
            System.out.println("UNWIND WHEN DONE TESTING!!!");
        }
    }

    // OLD, CHECK THAT STILL WORKS/REWRITE
    public void runWinch(double speed) {
        System.out.println("HEYY WINCH IS RUNNING AT SPEED " + winchMotor.getEncoder().getVelocity());
        winchMotor.set(Math.abs(speed));        
    }

    // OLD, CHECK THAT STILL WORKS/REWRITE
    public double getLiftHeight() {
        return liftEnc.getDistance();
    }

    // OLD, CHECK THAT STILL WORKS/REWRITE
    public double getWinchHeight() {
        return winchEnc.getPosition();
    }

    // OLD, CHECK THAT STILL WORKS/REWRITE
    public double getLiftSpeed() {
        return liftEnc.getRate();
    }

    // OLD, CHECK THAT STILL WORKS/REWRITE 
    public double getWinchSpeed() {
        return winchEnc.getVelocity();
    }

    public boolean isOnBar() {
        //returns true if the hook is up on the bar
        //TODO: is this necessary?
        return false;
    }

    public double getVoltage() {
        return powerDistributionPanel.getVoltage();
    }

    public boolean isButtonPressed() {
        boolean isButtonPressed = buttonCheck.get();
       return isButtonPressed;
    }

    public boolean isHookAttached() {
        boolean isHookAttached = checkHookAttached.get();
       return isHookAttached;
    }


}