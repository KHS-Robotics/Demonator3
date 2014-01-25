/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


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
public class RobotTestCode extends IterativeRobot 
{
    public static final int kForward_val = 1;
    public static final int kReverse_val = 2;
    private boolean driveGear;
    private double encLeftRate;
    private double encRightRate;
    private int numLoops=0;
    private boolean compressorStatus;
        
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
    Encoder encLeft = new Encoder(1,2,true);
    Encoder encRight = new Encoder(3,4,true);
    

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit () 
    {
            encLeft.start();
            encRight.start();
            driveGear=false; //low gear
            try 
            {
                cjag1=new CANJaguar(2);
                cjag2=new CANJaguar(3);
                //cjag3=new CANJaguar(4);
                //cjag4=new CANJaguar(5);
                //cjag5=new CANJaguar(6);
                //cjag6=new CANJaguar(7);
                compressor.set(Relay.Value.kOff);
            }
            catch (CANTimeoutException e)
            {
            }
            s1.set(DoubleSolenoid.Value.kReverse);
            //DigitalInput input=new DigitalInput(1);
    }
    
    /**
     * Enables the compressor if the user asks only if air is needed
     */
    public void compressorEnable ()
    {
        if (j2.getRawButton(4) == true && false == pSwitch.get())
        {
            compressor.set(Relay.Value.kForward);
            compressorStatus=true;
            
        }
        else
        {
            compressor.set(Relay.Value.kOff);
            compressorStatus=false;
        }
    }
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic () 
    {
        
    }
    
    /**
     * Updates driver station every 10 loops
     */
    public void updateDS()
    {
            
            if (driveGear)
            {
                driveStation.println(Line.kUser1,1,"HIGH GEAR!"); 
            }
            else
            {
                driveStation.println(Line.kUser1, 1,"LOW GEAR!");
            }
            if (compressorStatus)
            {
                driveStation.println(Line.kUser2,2,"Compressor on");
            }
            else
            {
                driveStation.println(Line.kUser2,2,"Compressor off");
            }
            driveStation.println(Line.kUser3,3,"Left encoder rate"+encLeft.getRate()+"Right encoder rate"+encRight.getRate());
   }
    
    /**
     * Teleop driving method
     */
        
    public void driveTeleop()
    {
        
        encLeftRate = encLeft.getRate();
        encRightRate = encRight.getRate();
        if (j1.getRawButton(3) == true) 
        {
            s1.set(DoubleSolenoid.Value.kForward);
            driveGear=true;
            
        }
        
        if (j1.getRawButton(4) == true) 
        {
            s1.set(DoubleSolenoid.Value.kReverse);
            driveGear=false;
        }
        
        driveStation.updateLCD();

        compressorEnable();
        
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
            cjag1.setX(left,(byte)1);
            cjag2.setX(right,(byte)1);
            //cjag3.setX(left,(byte)1);
            //cjag4.setX(right,(byte)1);
            //cjag5.setX(right,(byte)1);
            //cjag6.setX(right,(byte)1);
            CANJaguar.updateSyncGroup((byte)1);
        }
        catch (CANTimeoutException e)
        {    
        }
    }
    

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic () 
    {
        numLoops++;
        
        driveTeleop();
        
        
        
        
        
        if (numLoops%10==0)
            updateDS();
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic () 
    {
    
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
