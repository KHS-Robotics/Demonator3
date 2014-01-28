/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------*/
/* Kennett High School Demon Robotics souce code for FRC 2014 Game */
/*-----------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CANJaguar;
//import edu.wpi.first.wpilibj.Jaguar;
//import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.DriverStationLCD.Line;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.DigitalOutput;
//import edu.wpi.first.wpilibj.DigitalModule;

//comment


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
    public static final double encMultiplier = .0014; //each encoder pulse translates to .0014 ft
    private int numLoops=0;
    private boolean driveGear;
    private boolean compressorStatus;
    
    CANJaguar driveLeft1;
    CANJaguar driveLeft2;
    CANJaguar armJag;
    CANJaguar driveLeft3;
    CANJaguar driveRight1;
    CANJaguar driveRight2;
    CANJaguar driveRight3;
    
    DoubleSolenoid s1 = new DoubleSolenoid(kForward_val, kReverse_val);
    
    DriverStationLCD driverStation = DriverStationLCD.getInstance();
    
    Relay compressor = new Relay(1, Relay.Direction.kForward);
    
    DigitalInput pSwitch = new DigitalInput(14);
    DigitalInput limitSwitch = new DigitalInput(9);
    
    Joystick j1 = new Joystick(1);
    Joystick j2 = new Joystick(2);
    
    Encoder encLeft = new Encoder(1,2);
    Encoder encRight = new Encoder(3,4);
   

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit () 
    {
            encLeft.start();
            encRight.start();
            
            compressorStatus = true; //pSwitch.get();
            
            driveGear = false; //determines which gear the transmission is currently in False=low
            try 
            {
                driveLeft1=new CANJaguar(2);
                driveLeft2=new CANJaguar(3);
 //               armJag = new CANJaguar(8);
                driveLeft3=new CANJaguar(4);
                driveRight1=new CANJaguar(5);
                driveRight2=new CANJaguar(6);
                driveRight3=new CANJaguar(7);
                compressor.set(Relay.Value.kOff);
            }
            catch (CANTimeoutException e)
            {
            }
            s1.set(DoubleSolenoid.Value.kReverse);
    }
    
    /**
     * Enables the compressor if the user asks only if air is needed
     */
    public void compressorEnable ()
    {
        if (compressorStatus == true && false == pSwitch.get())
        {
            compressor.set(Relay.Value.kForward);
        }
        else
        {
            compressor.set(Relay.Value.kOff);
        }
    }
    /**
     * This function is called periodically during autonomous
     */
    public void updateDS()
    {
        if (driveGear)
        {
            driverStation.println(Line.kUser1,1,"HIGH GEAR!");
        }
        else
        {
            driverStation.println(Line.kUser1, 1,"LOW GEAR!");
        }
        
        if (compressorStatus)
        {
            driverStation.println(Line.kUser2,2,"Compressor Enabled");
        }
        else
        {
            driverStation.println(Line.kUser2,2,"Compressor Disabled");
        }
        
//        try
//        {
//            driverStation.println(Line.kUser4, 4, "" + driveLeft1.getSpeed() + " " + driveLeft2.getSpeed());
//        }
//        catch(CANTimeoutException e)
//        {}
        
        driverStation.println(Line.kUser3,3,"Left encoder rate "+encLeft.getRate()+" Right encoder rate"+encRight.getRate());
        driverStation.updateLCD();
    }
    public void autonomousPeriodic () 
    {

    }
    
    public void driveTeleop()
    {
        if (j1.getRawButton(3) == true) 
        {
            s1.set(DoubleSolenoid.Value.kForward);
            driveGear = true;
            
        }
        
        if (j1.getRawButton(4) == true) 
        {
            s1.set(DoubleSolenoid.Value.kReverse);
            driveGear = false;
        }
   
        //armControl();
        
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
            driveLeft1.setX(left,(byte)1);
            driveLeft2.setX(left,(byte)1);
            driveLeft3.setX(left,(byte)1);
            driveRight1.setX(right,(byte)1);
            driveRight2.setX(right,(byte)1);
            driveRight3.setX(right,(byte)1);
            CANJaguar.updateSyncGroup((byte)1);
            double a = driveLeft1.getSpeed();
            double b = driveLeft2.getSpeed();
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
        driveTeleop();
        compressorEnable();
        
        if (numLoops % 10 == 0)
            updateDS();
        
        numLoops++;
    }
    
    /**
     * This method is for controlling the arm of the robot
     */
    /*
    public void armControl ()
    {
        try 
        {
            double axisY = j2.getAxis(Joystick.AxisType.kY);
            double encCount = encLeft.getDistance();
            int setPoint1 = 50, setPoint2 = 100, setPoint3 = 150, setPoint4 = 200;
            driverStation.println(Line.kUser4, 4, "" + encCount);
        
            if (j2.getRawButton(1) == true)
            {
                if (encCount < setPoint1)
                {
                    armJag.setX(1);
                }
                else if (encCount > setPoint1)
                {
                    armJag.setX(-1);
                }
            }
        
            if (j2.getRawButton(2) == true)
            {
                if (encCount < setPoint2)
                {
                    armJag.setX(1);
                }
                else if (encCount > setPoint2)
                {
                    armJag.setX(-1);
                }
            }
        
            if (j2.getRawButton(3) == true)
            {
                if (encCount < setPoint3)
                {
                    armJag.setX(1);
                }
                else if (encCount > setPoint3)
                {
                    armJag.setX(-1);
                }
            }
        
            if (j2.getRawButton(4) == true)
            {
                if (encCount < setPoint4)
                {
                    armJag.setX(1);
                }
                else if (encCount > setPoint4)
                {
                    armJag.setX(-1);
                }
            }
        
            if (limitSwitch.get() == true && axisY > 0)
            {
                axisY = 0;
                {
                   armJag.setX(axisY);
                }
            }
        }
        catch (CANTimeoutException)
        {
        }
    }
    */
    
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