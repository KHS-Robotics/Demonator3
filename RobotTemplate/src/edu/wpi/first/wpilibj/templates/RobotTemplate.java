/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;

//test to see if git works
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.Jaguar;
//import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.DriverStationLCD.Line;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Relay;
//import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.DigitalOutput;
//import edu.wpi.first.wpilibj.DigitalModule;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends IterativeRobot 
{
    public static final int kForward_val = 1;
    public static final int kReverse_val = 2;
    private boolean highGear;
    private boolean enableCompressor;
        
    CANJaguar cjag1;
    CANJaguar cjag2;
    //CANJaguar cjag3;
    //CANJaguar cjag4;
    //CANJaguar cjag5;
    //CANJaguar cjag6;
    DoubleSolenoid s1 = new DoubleSolenoid(kForward_val, kReverse_val);
    DriverStationLCD driveStation = DriverStationLCD.getInstance();
    Relay compressor = new Relay(1, Relay.Direction.kForward);
    DigitalInput pSwitch = new DigitalInput(14);
    Joystick j1 = new Joystick(1);
    Joystick j2 = new Joystick(2);
    Encoder enc1 = new Encoder(1,2);
    Encoder enc2 = new Encoder(3,4);

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit () 
    {
            enc1.start();
            enc2.start();
            
            compressor.set(Relay.Value.kOff);
            
            highGear = false; //determines which gear the transmission is currently in
            try
            {
                cjag1 = new CANJaguar(2);
                cjag2 = new CANJaguar(3);
                //cjag3=new CANJaguar(4);
                //cjag4=new CANJaguar(5);
                //cjag5=new CANJaguar(6);
                //cjag6=new CANJaguar(7);
            }
            catch (CANTimeoutException e)
            {
            }
            s1.set(DoubleSolenoid.Value.kReverse);
            //DigitalInput input=new DigitalInput(1);
    }
    
    /**
     * This function is run when the robot is first started up in
     * autonomous and should be used for any initialization code. 
     */
    public void autonomousInit ()
    {
        enableCompressor = false;
        compressor.set(Relay.Value.kOn);
    }
    
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic () 
    {

    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic () 
    {
        double e1rate = enc1.getRate();
        double e2rate = enc2.getRate();
        if (j1.getRawButton(3) == true) 
        {
            s1.set(DoubleSolenoid.Value.kForward);
            driveStation.println(Line.kUser1,1,"HIGH GEAR!");
        }
        
        if (j1.getRawButton(4) == true) 
        {
            s1.set(DoubleSolenoid.Value.kReverse);
            driveStation.println(Line.kUser1, 1,"LOW GEAR!");
        }
        
        driveStation.updateLCD();

        compressorControl();
        
        double x = j1.getX();
        double y = j1.getY();
        double left = y-x;
        double right = y+x;
        if (left > 1.0)
        {
            left = 1.0;
        }
        else if (left < -1.0)
        {
            left = -1.0;
        }
        if (right > 1.0)
        {
            right = 1.0;
        }
        else if (right < -1.0)
        {
            right = -1.0;
        }
        try 
        {
            cjag1.setX(left,(byte) 1);
            cjag2.setX(right,(byte) 1);
            //cjag3.setX(left,(byte) 1);
            //cjag4.setX(right,(byte) 1);
            //cjag5.setX(right,(byte) 1);
            //cjag6.setX(right,(byte) 1);
            CANJaguar.updateSyncGroup((byte) 1);
            double a = cjag1.getSpeed();
            double b = cjag2.getSpeed();
            driveStation.println(Line.kUser3,3,"" + b + "" + a);
        }
        catch (CANTimeoutException e)
        {    
        }
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic () 
    {
    
    }
    
    /**
     * Enables the compressor if the user asks only if air is needed
     */
    public void compressorControl ()
    {
        if (j2.getRawButton(4) == true && false == pSwitch.get())
        {
            compressor.set(Relay.Value.kForward);
            enableCompressor = true;
            driveStation.println(Line.kUser2,2,"Compressor on");
        }
        else
        {
            compressor.set(Relay.Value.kOff);
            enableCompressor = false;
            driveStation.println(Line.kUser2,2,"Compressor off");
        }
    }
    
    public void disabledInit ()
    {

    }
    
    /**
     * This function is called periodic during disabled mode
     */
    public void disabledPeriodic() 
    {
        
    }
}
