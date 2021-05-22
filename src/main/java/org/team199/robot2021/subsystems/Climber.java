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
import edu.wpi.first.wpilibj.DigitalInput;


public class Climber extends SubsystemBase {
    private static final double kLiftConversionFactor =  1.0/12 * 2 * Math.PI * 115; //pulse resolution * 2pi * gear ratio = radians lift motor has travelled TODO find r
    private static final double kWinchConversionFactor = 1.0/42 * 2 * Math.PI * 0.5 * 13.2; //pulse resolution * 2*pi*r * gear ratio (1.0/42 * r * 2 * pi) = distance of rope extended

    //TODO: find good values and then set to final
    public static double kLiftDeploySpeed = 0.3; //TODO: set correct speed (test empirically)
    public static double kWinchDeploySpeed = 1; //No longer used
    public static double kLiftLowerSpeed = -0.1; //Vaguely arbitrary, but also not important might wanna ask others TODO : Name this better
    public static double kLiftKeepSpeed = 0.06; //TODO: set correct speed (trial and error)
    public static double kLiftRetractSpeed = -0.001; //TODO: name this better
    public static double kWinchRetractSpeedFirst = 0; //TODO: figure out how bungie cord will affect this
    //the winch doesn't need to retract before the hook reaches the bar because any slack is taken on by the rollers
    public static double kWinchRetractSpeedSecond = 1;
    //speed of winch retracting after hook attaches 
    public static double kArmRetryDistance = 1.046; //height arm goes to after going to max height
    //public static double kLiftAdjustSpeed = 0.2; //vestigial from perevious iteration
    //public static double kWinchAdjustSpeed = 0.2; //vestigial from perevious iteration
    //public static double kHighCurrent = 83; //based on current budget? may have to be changed (number is in amps?)
    //public static double kLowCurrent = 0.2;

    

    public static final double kLiftTallHeight = 1.803;
    public static final double kLiftShortHeight = 0.4754; //maybe check this value
    public static final double kLiftLowerHeight = 0;  //TODO: set this one probably (could be double max height depending on if scalar or vector)
    public static final double kWinchMaxHeight = 59; //Winch height for fully extended arm
    //public static final double kWinchMaxHeight = 59; //Winch height for fully extended arm (no longer necessary due to how lift raises)
    public static final double kWinchStartHeight = 0; //Winch height for arm before extension
    public static final double kWinchFinalHeight = -80; //No longer necessary
    public boolean isFinishedDeploying = false;
    // Winch height for after extension
    private final WPI_VictorSPX liftMotor = MotorControllerFactory.createVictor(Constants.DrivePorts.kClimberLift); //this is a 775
    private final CANSparkMax winchMotor = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kClimberWinch);
    private final Encoder liftEnc = new Encoder(2, 3);
    private final CANEncoder winchEnc = winchMotor.getEncoder();
    private final PowerDistributionPanel powerDistributionPanel = new PowerDistributionPanel(1); //TODO: set correct module
    private final Joystick placeholder = new Joystick(1);    
    private final Joystick placeholder2 = new Joystick(2);
    private final DigitalInput checkHookAttached = new DigitalInput(2);
    public Climber(){
        liftEnc.setDistancePerPulse(kLiftConversionFactor);
        winchEnc.setPositionConversionFactor(kWinchConversionFactor);
        winchEnc.setVelocityConversionFactor(kWinchConversionFactor / 60);
        liftEnc.reset();
        winchEnc.setPosition(kWinchFinalHeight);
        winchMotor.setIdleMode(IdleMode.kCoast);
       // liftMotor.setSmartCurrentLimit(30);

        SmartDashboard.putNumber("Climber.kLiftDeploySpeed", kLiftDeploySpeed);
        SmartDashboard.putNumber("Climber.kWinchDeploySpeed", kWinchDeploySpeed);
        SmartDashboard.putNumber("Climber.kLiftKeepSpeed", kLiftKeepSpeed);
        SmartDashboard.putNumber("Climber.kLiftRetractSpeed", kLiftRetractSpeed);
        SmartDashboard.putNumber("Climber.kWinchRetractSpeedFirst", kWinchRetractSpeedFirst);
        // SmartDashboard.putNumber("Climber.kLiftAdjustSpeed", kLiftAdjustSpeed);
    }

    public void periodic()  {
        kLiftDeploySpeed = SmartDashboard.getNumber("Climber.kLiftDeploySpeed", kLiftDeploySpeed);
        kWinchDeploySpeed = SmartDashboard.getNumber("Climber.kWinchDeploySpeed", kWinchDeploySpeed);
        kLiftKeepSpeed = SmartDashboard.getNumber("Climber.kLiftKeepSpeed", kLiftKeepSpeed);
        kLiftRetractSpeed = SmartDashboard.getNumber("Climber.kLiftRetractSpeed", kLiftRetractSpeed);
        kWinchRetractSpeedFirst = SmartDashboard.getNumber("Climber.kWinchRetractSpeedFirst", kWinchRetractSpeedFirst);
        // kLiftAdjustSpeed = SmartDashboard.getNumber("Climber.kLiftAdjustSpeed", kLiftAdjustSpeed);
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

    // public double getCurrent() {
    //     return liftMotor.getOutputCurrent();
    // }

    //isHookAttached to arm, not bar 
    public boolean isHookAttached() {
        boolean isHookAttached = checkHookAttached.get();
       return isHookAttached;
    }

    public void setWinchIdleCoast(){
        winchMotor.setIdleMode(IdleMode.kCoast);
    } 

    public void setWinchIdleBrake(){
        winchMotor.setIdleMode(IdleMode.kBrake);
    }
}