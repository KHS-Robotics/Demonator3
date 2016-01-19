/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------*/
/* Kennett High School Demon Robotics source code for FRC 2014 Game "Aerial Assist". */
/*-----------------------------------------------------------------------------------*/

package IterativeRobots.OldCode;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.DriverStationLCD.Line;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Gyro;

//Imports for the camera to determine "hot" goal
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.CriteriaCollection;
import edu.wpi.first.wpilibj.image.NIVision;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;
import edu.wpi.first.wpilibj.image.RGBImage;

/**
 * This class contains Kennett High School Demon Robotics source code for FRC 2014's game
 * Aerial Assist. The robot weighs in at about 120lbs and was developed by a team of
 * around 30 people at Kennett High School in Kennett Square, Pennsylvania by FRC
 * Team 4342, Demon Robotics
 * 
 * @author Magnus Murray
 * @author Nathan Smith
 * @author Ernest Wilson
 * @author Brian Lucas
 * @author Steve Chapman
 * 
 * @see <a href = "http://www.demonrobotics4342.org/apps/photos/photo?photoid=190465555">
 * A picture of Team 4342's Robot </a>
 * @see <a href = "http://frc-manual.usfirst.org/viewItem/3"> The Official Aerial
 * Assist Game Manual </a>
 * @see <a href = "https://www.youtube.com/watch?v=oxp4dkMQ1Vo"> A video explaining how to play Aerial Assist </a>
 */
public class RobotTemplate extends IterativeRobot 
{
    /**
     * Each encoder pulse translates to .0014 ft
     */
    public static final double DRIVE_ENC_MULT = .0014;

    /**
     * Value to determine if the driver is allowed to shift gears
     */
    public static final double MIN_SHIFT_SPEED = 0;

    /**
     * First preset encoder values for arm movement
     */
    public static final int ARM_PRESET1 = 0;

    /**
     * Second preset encoder values for arm movement
     */
    public static final int ARM_PRESET2 = 105;

    /**
     * Third preset encoder values for arm movement
     */
    public static final int ARM_PRESET3 = 128;

    /**
     * Fourth preset encoder values for arm movement
     */
    public static final int ARM_PRESET4 = 341;

    /**
     * Dead band for arm location
     */
    public static final int ARM_SETPOINT_DEADBAND = 25;
    
    /**
     * Proportional constant for arm movement.
     */
    public static final double ARM_KP = .000889; 

    /**
     * Dead band to determine if arm will be auto controlled with buttons
     */
    public static final double ARM_JOYSTICK_DEADBAND = .1; 

    /**
     * Number of pulses to cock the shooting mechanism
     */
    public static final int COCKING_PULSES = 5000;

    /**
     * Number of pulses to fire the shooting mechanism
     */
    public static final int FIRE_PULSES = 2500;

    /**
     * Variable used to determine if driver is intentionally moving
     */
    public static final double TURNING_DEADBAND = 0.05;

    /**
     * Reduction factor for gyro correction for left wheels
     */
    public static final double LEFT_MOTOR_DIVIDER = 0.9;

    /**
     * Reduction factor for gyro correction for right wheels
     */
    public static final double RIGHT_MOTOR_DIVIDER = 0.9;
    
    /**
     * Number of encoder pulses to go forward to desired area in autonomous
     */
    public static final int MAX_ENC_VAL = 21500;
    
    //Variables used to determine of goal is "hot"
    final int Y_IMAGE_RES = 480;
    final double VIEW_ANGLE = 37.4; //Axis M1011 camera
    final double PI = 3.141592653;
    final int RECTANGULARITY_LIMIT = 40;
    final int ASPECT_RATIO_LIMIT = 55;
    final int TAPE_WIDTH_LIMIT = 50;
    final int VERTICAL_SCORE_LIMIT = 50;
    final int LR_SCORE_LIMIT = 50;
    final int AREA_MINIMUM = 150;
    final int MAX_PARTICLES = 8;
    
    private int numLoops = 0; //loops used to update driver station
    private int numLoopsAutoFire = 1;
    private int autoStep; //used for autonomous, determines which step of routine to do
    private int autoAccumulate;
    private boolean driveGear; //variable used to determine if robot is in high or low gear
    private boolean compressorStatus; //variable used to determine if compressor is on or off
    private boolean armAutoControl; //variable used to determine if robot will be controller manually or with buttons
    private boolean hot; //variable used to determine if goal is "hot"
    private byte shooterState; //1 = loaded, 2 firing, 3 fired, 4 loading, state of robot's shooting mechanism
    private byte autoRoutine; //variable to determine which routine robot will execute in autonomous
    private double firingEncoderPulses; //variable to identify number of pulses mechanism has done for shooting
    private static int armSetpoint; //variable used to determine where the arm will move
    private int shootingTime; //timer used to shoot if the encoder breaks
    private int whichCANTimeout; //used to determine which CAN timed out //1-6 = drive train upon start(numbered), 8 = drive train(unspecified), 2 = arm
    private boolean canTimeout; //used to determine if a CANJaguar timed out
    
    AxisCamera camera; //the axis camera object (connected to the switch)
    CriteriaCollection cc; //the criteria for doing the particle filter operation
    
    Joystick j1 = new Joystick(1); //Joystick plugged into port 1
    Joystick j2 = new Joystick(2); //"Joystick" or box of switches plugged into port 2
    Joystick j3 = new Joystick(3); //Joystick plugged into port 3
    
    //All CANJaguars used for the robot
    CANJaguar driveLeft1, driveLeft2, driveLeft3, driveRight1, driveRight2, driveRight3, armJaguar; //CANJaguars used for drive train and arm
    
    Victor accum = new Victor(6); //Victor motor for accumulator to take in or spit out ball
    Victor firingMotor = new Victor(4); //Victor motor to control firing mechanism
    
    DoubleSolenoid s1 = new DoubleSolenoid(1, 2); //DoubleSolenoid to switch gears
    Solenoid accumSolenoid = new Solenoid(3); //Solenoid to move the accumulator wheel up or down
    Solenoid demonEyes = new Solenoid(4); //controls the lights
    
    DriverStationLCD driverStation = DriverStationLCD.getInstance(); //Driver Station to operate robot
    
    Relay compressor = new Relay(1, Relay.Direction.kForward); //compressor to switch gears, only goes forward
    
    DigitalInput limitSwitchTop = new DigitalInput(5); //bottom limit switch for the arm
    DigitalInput limitSwitchBottom = new DigitalInput(6); //top limit switch for the arm
    DigitalInput cockingLimitSwitch = new DigitalInput(7); //limit switch for the cocking mechanism
    DigitalInput pSwitch = new DigitalInput(14); //limit switch for the compressor (Stops at ~120 PSI)
    
    Encoder armEncoder = new Encoder(1, 2); //encoder to measure the pulses for the arm
    Encoder firingEncoder = new Encoder(3, 4); //encoder to measure the pulses for the shooting mechanism
    Encoder encLeft = new Encoder(8, 9); //encoder to measure revolutions of left wheels
    Encoder encRight = new Encoder(10, 11); //encoder to measure revolutions of right wheels
    
    Gyro gyro = new Gyro(1); //gyro used to measure stability of robot
    
    private boolean debug;
    
    /**
     * Nested class to determine "hot" goal.
     */
    public class Scores 
    {
        double rectangularity;
        double aspectRatioVertical;
        double aspectRatioHorizontal;
    }

    /**
     * Nested class to determine "hot" goal.
     */
    public class TargetReport 
    {
        int verticalIndex;
        int horizontalIndex;
        boolean Hot;
        double totalScore;
        double leftScore;
        double rightScore;
        double tapeWidthScore;
        double verticalScore;
    }
   
    ////////////////////BELOW ARE OVERRIDED METHODS FROM ITERATIVE ROBOT/////////////////////////
    /**
     * This method is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit () 
    {
        encLeft.setDistancePerPulse(DRIVE_ENC_MULT);
        encLeft.start();
        encRight.setDistancePerPulse(DRIVE_ENC_MULT);
        encRight.start();
        firingEncoder.start();
        armEncoder.start();
        demonEyes.set(true);
        compressorStatus = true;
        driveGear = false;
        armAutoControl = false;
        gyro.setSensitivity(0.007);
        
        if (cockingLimitSwitch.get())
        {
            shooterState = 1;
        }
        else
        {
            shooterState = 3;
        }
        
        camera = AxisCamera.getInstance("10.43.42.11");
        
        try { driveLeft1 = new CANJaguar(2); }
        catch (CANTimeoutException ex) { canTimeout = true; whichCANTimeout = 1; }
        try { driveLeft2 = new CANJaguar(2); }
        catch (CANTimeoutException ex) { canTimeout = true; whichCANTimeout = 2; }
        try { driveLeft3 = new CANJaguar(2); }
        catch (CANTimeoutException ex) { canTimeout = true; whichCANTimeout = 3; }
        try { driveRight1 = new CANJaguar(2); }
        catch (CANTimeoutException ex) { canTimeout = true; whichCANTimeout = 4; }
        try { driveRight2 = new CANJaguar(2); }
        catch (CANTimeoutException ex) { canTimeout = true; whichCANTimeout = 5; }
        try { driveRight3 = new CANJaguar(2); }
        catch (CANTimeoutException ex) { canTimeout = true; whichCANTimeout = 6; }
        
        compressor.set(Relay.Value.kOff);
        s1.set(DoubleSolenoid.Value.kReverse);
    }
    
    /**
     * This method is run when the robot is first started up in
     * teleop and should be used for any initialization code.
     */
    public void teleopInit ()
    {
        numLoops = 0;
        shootingTime = 0;
        if (cockingLimitSwitch.get())
        {
            shooterState = 1;
        }
        else
        {
            shooterState = 3;
        }
    }
    
    /**
     * This method is run when the robot is first started up in
     * autonomous and should be used for any initialization code. 
     */
    public void autonomousInit ()
    {
        autoStep = 0;
        shooterState = 1;
        autoAccumulate = 0;
        numLoops = 0;
        compressorStatus = true;
        hot = false;
        cc = new CriteriaCollection();
        cc.addCriteria(NIVision.MeasurementType.IMAQ_MT_AREA, AREA_MINIMUM, 65535, false);
        autoRoutines();
        
        if (compressorStatus && !pSwitch.get())
        {
            compressor.set(Relay.Value.kForward);
        }
        else
        {
            compressor.set(Relay.Value.kOff);
        }
    }
    
    /**
     * This method is called periodically during autonomous control.
     */
    public void autonomousPeriodic () 
    {
        switch (autoRoutine)
        {
            case 1 : 
                autoRoutineShootWithCamera();
                break;
                
            case 2 :
                onlyMoveAuto();
                break;
                
            case 3 :
                break;
                
            case 4 :
                break;
                
            case 5 :
                break;
                
            case 6 :
                break;
                
            case 7 : 
                break;
                
            case 8 :
                break;
                
            default :
                autoRoutineShootNoCamera();
                break;
        }
        if (numLoops % 10 == 0)
        {
            updateDS();
        }
        numLoops++;
    }
    
    /**
     * This method is called periodically during operator control/
     */
    public void teleopPeriodic () 
    {
        driveTeleop();
        compressorControl();
        firingControl();
        accumControl();
        armControl();
        resetEncoderVals();
        
        if (numLoops % 10 == 0)
        {
           updateDS();
        }
        numLoops++;
    }
    
    /**
     * This method is used when the robot is disabled in autonomous
     */
    public void autonomousDisabled ()
    {
        if (numLoops % 10 == 0)
        {
            gyro.reset();
        }
        numLoops++;
    }
    
    /**
     * This method is run when the robot is turned off and should be
     * used for any initialization code.
     */
    public void disabledInit ()
    {
        numLoops = 0;
        
        try { driveLeft1.configFaultTime(0.5); }
        catch (CANTimeoutException ex) { canTimeout = true; whichCANTimeout = 1; }
        try { driveLeft2.configFaultTime(0.5); }
        catch (CANTimeoutException ex) { canTimeout = true; whichCANTimeout = 2; }
        try { driveLeft3.configFaultTime(0.5); }
        catch (CANTimeoutException ex) { canTimeout = true; whichCANTimeout = 3; }
        try { driveRight1.configFaultTime(0.5); }
        catch (CANTimeoutException ex) { canTimeout = true; whichCANTimeout = 4; }
        try { driveRight2.configFaultTime(0.5); }
        catch (CANTimeoutException ex) { canTimeout = true; whichCANTimeout = 5; }
        try { driveRight3.configFaultTime(0.5); }
        catch (CANTimeoutException ex) { canTimeout = true; whichCANTimeout = 6; }
        try { armJaguar.configFaultTime(.05); }
        catch (CANTimeoutException ex) { canTimeout = true; whichCANTimeout = 7; }
        
        try { driveLeft1.configNeutralMode(CANJaguar.NeutralMode.kCoast); }
        catch (CANTimeoutException ex) { canTimeout = true; whichCANTimeout = 1; }
        try { driveLeft2.configNeutralMode(CANJaguar.NeutralMode.kCoast); }
        catch (CANTimeoutException ex) { canTimeout = true; whichCANTimeout = 2; }
        try { driveLeft3.configNeutralMode(CANJaguar.NeutralMode.kCoast); }
        catch (CANTimeoutException ex) { canTimeout = true; whichCANTimeout = 3; }
        try { driveRight1.configNeutralMode(CANJaguar.NeutralMode.kCoast); }
        catch (CANTimeoutException ex) { canTimeout = true; whichCANTimeout = 4; }
        try { driveRight2.configNeutralMode(CANJaguar.NeutralMode.kCoast); }
        catch (CANTimeoutException ex) { canTimeout = true; whichCANTimeout = 5; }
        try { driveRight3.configNeutralMode(CANJaguar.NeutralMode.kCoast); }
        catch (CANTimeoutException ex) { canTimeout = true; whichCANTimeout = 6; }
        try { armJaguar.configNeutralMode(CANJaguar.NeutralMode.kBrake); }
        catch (CANTimeoutException ex) { canTimeout = true; whichCANTimeout = 7; }
    }
    
    /**
     * This method is called periodic during disabled mode
     */
    public void disabledPeriodic() 
    {
        if (numLoops % 10 == 0)
        {
            gyro.reset();
        }
        numLoops++;
    }
    ////////////////////END OF OVERRIDED METHODS FROM ITERATIVE ROBOT////////////////////////////
    
    ////////////////// BELOW METHODS ARE MADE FOR TELEOP/////////////////////////////////////////
    /**
     * This method is for driving the robot.
     */
    public void driveTeleop()
    {
        debug = j2.getRawButton(7);
        
        double avgSpeed = (encLeft.getRate() + encRight.getRate ()) / 2 ;
        double leftGyroCorrection = gyroCorrectionLeft();
        double rightGyroCorrection = gyroCorrectionRight();
        if (Math.abs(gyro.getAngle()) > 15)
        {
            s1.set(DoubleSolenoid.Value.kReverse);
            driveGear = false; //Low gear
        }
        if (j1.getRawButton(11) && avgSpeed > MIN_SHIFT_SPEED && gyro.getAngle() < 15)
        {
            s1.set(DoubleSolenoid.Value.kForward);
            driveGear = true; //High gear
        }
        else
        {
            s1.set(DoubleSolenoid.Value.kReverse);
            driveGear = false; //Low gear
        }
        
        double x = j1.getX();
        double y = -j1.getY();
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
            driveLeft1.setX(left, (byte)1);
            driveLeft2.setX(left, (byte)1);
            driveLeft3.setX(left, (byte)1);
            driveRight1.setX(right, (byte)1);
            driveRight2.setX(right, (byte)1);
            driveRight3.setX(right, (byte)1);
            CANJaguar.updateSyncGroup((byte)1);
        }
        catch (CANTimeoutException ex)
        {
            canTimeout = true;
            whichCANTimeout = 8;
        }
    }
    
    /**
     * This method is for updating the Driver Station
     */
    public void updateDS()
    {
        driverStation.clear();
        
        if (isOperatorControl())
        {
            if (driveGear)
            {
                driverStation.println(Line.kUser1, 1, "HIGH GEAR!");
            }
            else
            {
                driverStation.println(Line.kUser1, 1, "LOW GEAR!");
            }

            if (shooterState == 1)
            {
                driverStation.println(Line.kUser2, 1, "Ready to Fire!");
            }
            else if (shooterState == 3)
            {
                driverStation.println(Line.kUser2, 1, "Reload!");
            }
            else if (shooterState == 2)
            {
                driverStation.println(Line.kUser2, 1, "Firing!");
            }
            else if (shooterState == 4)
            {
                driverStation.println(Line.kUser2, 1, "Reloading!!");
            }
            
            driverStation.println(Line.kUser3, 1, "TopLS: " + limitSwitchTop.get() + ", BotLS: " + limitSwitchBottom.get());
            driverStation.println(Line.kUser4, 1, "FireEnc: " + firingEncoder.getRaw() + ", ArmEnc: " + armEncoder.getRaw());
            driverStation.println(Line.kUser5, 1, "RER: " + encRight.getRaw() + ", LER: " + encLeft.getRaw());
        }
        else if (isAutonomous())
        {
            driverStation.println(Line.kUser1, 1, "AutoStep: " + autoStep);
            driverStation.println(Line.kUser2, 1, "Gyro Angle: " + gyro.getAngle());
            driverStation.println(Line.kUser3, 1, "LER: " + encLeft.getRaw()); //Left Encoder raw value
            driverStation.println(Line.kUser4, 1, "RER: " + encRight.getRaw()); //Right Encoder raw value
            driverStation.println(Line.kUser5, 1, "Hot: " + hot);
        }
        else if (isDisabled())
        {
            driverStation.println(Line.kUser1, 1, "Gyro Angle: " + gyro.getAngle());
        }
        
        if (canTimeout)
        {
            driverStation.println(Line.kUser6, 1, "CAN #" + whichCANTimeout + " timed out");
        }
        
        driverStation.updateLCD();
    }
    
    /**
     * This method is for enabling the compressor.
     */
    public void compressorControl ()
    {
        if (!j2.getRawButton(2) && !pSwitch.get())
        {
            compressor.set(Relay.Value.kForward);
        }
        else
        {
            compressor.set(Relay.Value.kOff);
        }
    }
    
    /**
     * This method is for controlling firing and reloading
     * 1 = loaded, 2 = firing, 3 = fired, 4 = loading
     */
    public void firingControl ()
    {
        firingEncoderPulses = firingEncoder.getRaw();
        
        if (j2.getRawButton(3))
        {
            if (j2.getRawButton(5))
            {
                firingMotor.set(1);
            }
            else if (j2.getRawButton(5))
            {
                firingMotor.set(0);
            }
        }
        else if (!j2.getRawButton(3))
        {
            if (shooterState == 1) //Loaded
            {
                if (j2.getRawButton(5) && j2.getRawButton(10))
                {
                    firingMotor.set(1);
                    shooterState = 2; //Firing
                }
                else
                {
                    firingMotor.set(0);
                }
            }
            else if (shooterState == 2) //If it's firing
            {
                shootingTime++;
                if (firingEncoderPulses >= FIRE_PULSES || 10 < shootingTime) 
                {
                    firingMotor.set(0);
                    shooterState = 3; //Fired
                    shootingTime = 0;
                }
                else
                {
                    firingMotor.set(1);
                }
            }
            else if (shooterState == 3) //If its been fired
            {
                if (!j2.getRawButton(5)) 
                {
                    firingMotor.set(1);
                    shooterState = 4; //Loading
                }
                else
                {
                     firingMotor.set(0);
                }
            }
            else if (shooterState == 4) //If it's Loading
            {
                shootingTime++;
                if (firingEncoderPulses >= COCKING_PULSES || cockingLimitSwitch.get() || 75 < shootingTime)
                {
                    firingMotor.set(0);
                    shooterState = 1; //Loaded
                    shootingTime = 0;
                    firingEncoder.reset();
                }
                else
                {
                    firingMotor.set(1);
                }
            }
            else
            {
                firingMotor.set(0);
                shootingTime = 0;
                shooterState = 1; //Loaded
            }
        }
    }
    
    /**
     * This method is for controlling the arm of the robot
     */
    public void armControl ()
    {
        double armAxis = j3.getY();
        
        int armEncoderPulses = armEncoder.getRaw();
        
        if (Math.abs(armAxis) > ARM_JOYSTICK_DEADBAND)
        {
            armAutoControl = false;
        }
        else if (j3.getRawButton(8))
        {
            armAutoControl = true;
            armSetpoint = ARM_PRESET1;
        }
        else if (j3.getRawButton(9))
        {
            armAutoControl = true;
            armSetpoint = ARM_PRESET2;
        }  
        else if (j3.getRawButton(10))
        {
            armAutoControl = true;
            armSetpoint = ARM_PRESET3;
        }
        else if (j3.getRawButton(11))
        {
            armAutoControl = true;
            armSetpoint = ARM_PRESET4;
        }
        
        if (armAutoControl)
        {
            int armError = armEncoderPulses - armSetpoint;
       
            if (armEncoderPulses < armSetpoint + ARM_SETPOINT_DEADBAND && armEncoderPulses > armSetpoint - ARM_SETPOINT_DEADBAND)
            {
                armAxis = 0;
            }
            else if (armEncoderPulses < armSetpoint - ARM_SETPOINT_DEADBAND)
            {
                armAxis = (ARM_KP * armError) - 0.2;
            }
            else if (armEncoderPulses > armSetpoint + ARM_SETPOINT_DEADBAND)
            {
                armAxis = (ARM_KP * armError) + 0.2;
            }
        }
        
        if (armAxis > 0 && limitSwitchTop.get())
        {
            armAxis = 0;
        }
        else if (armAxis < 0 && !limitSwitchBottom.get())
        {
            armAxis = 0;
        }
        
        if (armAxis > 1.0)
        {
            armAxis = 1.0;
        }
        else if (armAxis < -1.0)
        {
            armAxis = -1.0;
        }
        
        if (limitSwitchTop.get())
        {
            armEncoder.reset();
        }
        
        try
        {
            armJaguar.setX(armAxis);
        }
        catch (CANTimeoutException ex)
        {
            canTimeout = true;
            whichCANTimeout = 7;
        }
    }
    
    /**
     * This method works the feeder for the robot.
     * The purpose is to pull or push the ball for
     * shooting or releasing.
     */
    public void accumControl ()
    {
        if (j1.getRawButton(7))
        {
            accum.set(-1);
        }
        else if (j1.getRawButton(8))
        {
            accum.set(1);
        }
        else if (j2.getRawButton(4))
        {
            accum.set(-1);
        }
        else if (j3.getRawButton(7))
        {
            accum.set(1);
        }
        else
        {
            accum.set(0);
        }
        
        if (j2.getRawButton(1))
        {
            accumSolenoid.set(true);  //accumulator up
        }
        else if (j2.getRawButton(5) && (1 == shooterState || 2 == shooterState))
        {
            accumSolenoid.set(true); //accumulator up
        }
        else if (!j2.getRawButton(1))
        {
            accumSolenoid.set(false); //accumulator down
        }
    }
    
    /**
     * Uses gyro to slow left motor if robot is turning right unintentionally.
     * @return leftOutputDivider
     */
    public double gyroCorrectionLeft ()
    {
       double leftOutputDivider = 1.0;
       
       if (j1.getRawButton(10))
       {  
         if (Math.abs(j1.getX()) < TURNING_DEADBAND)  //driver is trying to go straight
            {
                if (gyro.getAngle() > 2) //robot is straying right
                {
                    leftOutputDivider = LEFT_MOTOR_DIVIDER;
                }
            }
       }
       if (Math.abs(j1.getX()) > TURNING_DEADBAND)
        {
            gyro.reset();  //sets new heading
        }
       
       return leftOutputDivider;
    }
    
     /**
      * Uses gyro to slow right motor if robot is turning left unintentionally.
      * @return rightOutputDivider
      */
    public double gyroCorrectionRight ()
    {
        double rightOutputDivider = 1.0;
        
        if (j1.getRawButton(10))
        {
            if (Math.abs(j1.getX()) < TURNING_DEADBAND) //driver is trying to go straight
            {
                if (gyro.getAngle() < -2) //robot is straying left
                {
                    rightOutputDivider = RIGHT_MOTOR_DIVIDER;
                }
            }
        }
        if (Math.abs(j1.getX()) > TURNING_DEADBAND)
        {
            gyro.reset();  //sets new heading
        }
        
        return rightOutputDivider;
    }
    
    /**
     * This method is to reset all encoder values
     */
    public void resetEncoderVals ()
    {
        if (j2.getRawButton(9))
        {
            armEncoder.reset();
            firingEncoder.reset();
            encLeft.reset();
            encRight.reset();
        }
    }
    //////////////////END OF METHODS MADE FOR TELEOP////////////////////////////////////
    
    /////////////////BELOW ARE METHODS MADE FOR AUTONOMOUS//////////////////////////////
    /**
     * This method is one auto routine that simply moves 
     * forward and shoots
     */
    public void autoRoutineShootNoCamera ()
    {
        if (autoStep == 0)
        {
            if (autoAccum())
            {
                autoStep++;
            }
        }
        else if (autoStep == 1)
        {
            if (autoArmControl())
            {
                autoStep++;
            }
        }
        else if (autoStep == 2)
        {
            if (autoMove())
            {
                autoStep++;
            }
        }
        else if (autoStep == 3)
        {
            if (autoFire())
            {
                autoStep++;
            }
        }
    }
    
    /**
     * This method is another auto routine that moves forward and
     * used hot or not detection.
     */
    public void autoRoutineShootWithCamera ()
    {
        if (autoRoutine == 0)
        {
            if (autoAccum())
            {
                autoStep++;
            }
        }
        else if (autoRoutine == 1)
        {
            if (targetAnalysis())
            {
                autoStep++;
            }
        }
        else if (autoStep == 2)
        {
            if (autoArmControl() && autoMove())
            {
                numLoops = 0;
                autoStep++;
            }
        }
        else if (autoStep == 3)
        {
            numLoops++;
            if (hot)
            {
                if (autoFire())
                {
                    autoStep++;
                }
            }
            else
            {
                if (numLoops >= 150)
                {
                    if (autoFire())
                    {
                        autoStep++;
                    }
                }
            }
        }
    }
    
    /**
     * This method is an auto routine that only moves forward
     */
    public void onlyMoveAuto ()
    {
        if (autoStep == 0)
        {
            if (autoMove())
            {
                autoStep++;
            }
        }
    }
    
    /**
     * This method is for auto firing in autonomous.
     * @return true if finished, false otherwise
     */
    public boolean autoFire ()
    {
        
        firingEncoderPulses = firingEncoder.getRaw();
        
        if (shooterState == 1) //Loaded
        {
            if (numLoops > numLoopsAutoFire)
            {
                accumSolenoid.set(true);
                numLoopsAutoFire = numLoops+25;
                return false;
            }
            else if (numLoops >= (numLoopsAutoFire-15))
            {
                 accumSolenoid.set(true);
                 accum.set(1);
                 return false;
            }
            else if (numLoops >= (numLoopsAutoFire-25))
            {
                 accumSolenoid.set(true);
                 return false;
            }
            else if (numLoops == numLoopsAutoFire)
            {
                numLoopsAutoFire = 0;
                accumSolenoid.set(true);
                accum.set(1);
                firingMotor.set(1);
                shooterState = 4;
                return false;
            }
        }
        else if (shooterState == 4) //If it's Loading
        {
            if (firingEncoderPulses >= COCKING_PULSES || cockingLimitSwitch.get() || 75 < shootingTime)
            {
                 accumSolenoid.set(false);
                 accum.set(0);
                 firingMotor.set(0);
                 shooterState = 1;
                 return true;
            }
            else
            {
                 accumSolenoid.set(true);
                 accum.set(1);
                 firingMotor.set(1);
                 return false;
            }
        }
        else
        {
            firingMotor.set(0);
            shooterState = 1; //Loaded
            return false;
        }
        return false;
    }
    
    /**
     * This method controls the arm automatically during autonomous
     * @return true if finished, false otherwise
     */
    public boolean autoArmControl ()
    {
        boolean isFinished = false;
        armSetpoint = ARM_PRESET2;
        double armAxis = 0.0;
        int armEncoderPulses = armEncoder.getRaw();
        int armError = armEncoderPulses - armSetpoint;

        if (armEncoderPulses < armSetpoint + ARM_SETPOINT_DEADBAND && armEncoderPulses > armSetpoint - ARM_SETPOINT_DEADBAND )
        {
            armAxis = 0;
        }
        else if (armEncoderPulses < armSetpoint - ARM_SETPOINT_DEADBAND)
        {
            armAxis = (ARM_KP * armError) + 0.2;
        }
        else if (armEncoderPulses > armSetpoint + ARM_SETPOINT_DEADBAND)
        {
            armAxis = (ARM_KP * armError) - 0.2;
        }
        if (armAxis > 1)
        {
            armAxis = 1;
        }
        else if (armAxis < -1)
        {
            armAxis = -1;
        }
        if (armAxis < 0 && limitSwitchTop.get())
        {
            armAxis = 0;
        }
        else if (armAxis > 0 && !limitSwitchBottom.get())
        {
            armAxis = 0;
        }
        try
        {
            armJaguar.setX(armAxis);
        }
        catch(CANTimeoutException ex)
        {
            canTimeout = true;
            whichCANTimeout = 7;
        }
        if (armEncoderPulses == armSetpoint + ARM_SETPOINT_DEADBAND || armEncoderPulses == armSetpoint - ARM_SETPOINT_DEADBAND)
        {
            isFinished = true;
        }
        return isFinished;     
    }
    
    /**
     * This method is for auto accumulating the ball in auto
     * @return true if finished, false otherwise
     */
    public boolean autoAccum ()
    {
        autoAccumulate++;
        if (autoAccumulate < 50)
        {
            accum.set(-1);
            return false;
        }
        else
        {
            accum.set(0);
            return true;
        }
    }
    
    /**
     * This method is for moving the robot automatically in autonomous
     * @return true if finished moving, false otherwise
     */
    public boolean autoMove ()
    {
        double rightGyroCorrection = 1.0;
        double leftGyroCorrection = 1.0;
       
        if (gyro.getAngle() > 10)
        {
            leftGyroCorrection = 0.8;
        }
        else if (gyro.getAngle() < -10)
        {
            rightGyroCorrection = 0.8;
        }
        
        if (encLeft.get() < MAX_ENC_VAL && MAX_ENC_VAL > encRight.get())
        {
            try
            {
                driveLeft1.setX(.6 * leftGyroCorrection);
                driveLeft2.setX(.6 * leftGyroCorrection);
                driveLeft3.setX(.6 * leftGyroCorrection);
                driveRight1.setX(.6 * rightGyroCorrection);
                driveRight2.setX(.6 * rightGyroCorrection);
                driveRight3.setX(.6 * rightGyroCorrection);
            }
            catch (CANTimeoutException ex)
            {
                canTimeout = true;
                whichCANTimeout = 8;
            }
            return false;
        }
        else
        {
            try
            {
                driveLeft1.setX(0);
                driveLeft2.setX(0);
                driveLeft3.setX(0);
                driveRight1.setX(0);
                driveRight2.setX(0);
                driveRight3.setX(0);
            }
            catch (CANTimeoutException ex)
            {
                canTimeout = true;
                whichCANTimeout = 8;
            }
            return true;
        }
    }
    
    /**
     * This method is to determine the auto routine for autonomous
     * based on the digital inputs on the driver station.
     */
    public void autoRoutines ()
    {
        if (!isAutonomous())
        {
            if (DriverStation.getInstance().getDigitalIn(1))
            {
                autoRoutine = 1;
            }
            else if (DriverStation.getInstance().getDigitalIn(2))
            {
                autoRoutine = 2;
            }
            else if (DriverStation.getInstance().getDigitalIn(3))
            {
                autoRoutine = 3;
            }
            else if (DriverStation.getInstance().getDigitalIn(4))
            {
                autoRoutine = 4;
            }
            else if (DriverStation.getInstance().getDigitalIn(5))
            {
                autoRoutine = 5;
            }
            else if (DriverStation.getInstance().getDigitalIn(6))
            {
                autoRoutine = 6;
            }
            else if (DriverStation.getInstance().getDigitalIn(7))
            {
                autoRoutine = 7;
            }
            else if (DriverStation.getInstance().getDigitalIn(8))
            {
                autoRoutine = 8;
            }
            else
            {
                autoRoutine = 9;
            }
        }
    }
    //////////////////END OF METHODS MADE FOR AUTONOMOUS/////////////////////////////////
    
    /////////////BELOW ARE METHODS USED TO DETERMINE HOT GOAL USING NIVISION/////////////
    /////////////METHODS ARE COURTESY OF NATIONAL INSTRUMENTS (NI)///////////////////////
    /**
     * This method is for the camera to determine if the goal is "hot"
     * 
     * Method uses NIVision to find rectangles in the scene that are illuminated
     * by a LED ring light (similar to the model from FIRSTChoice). The camera sensitivity
     * is set very low so as to only show light sources and remove any distracting parts
     * of the image.
     * 
     * The CriteriaCollection is the set of criteria that is used to filter the set of
     * rectangles that are detected. In this example we're looking for rectangles with
     * a minimum width of 30 pixels and maximum of 400 pixels.
     * 
     * The algorithm first does a color threshold operation that only takes objects in the
     * scene that have a bright blue color component. Then a small object filter
     * removes small particles that might be caused by blue reflection scattered from other 
     * parts of the scene. Finally all particles are scored on rectangularity, and aspect ratio,
     * to determine if they are a target.
     * @return True if finished, false otherwise
     */
    public boolean targetAnalysis ()
    {
        TargetReport target = new TargetReport();
        int verticalTargets[] = new int[MAX_PARTICLES];
        int horizontalTargets[] = new int[MAX_PARTICLES];
        int verticalTargetCount, horizontalTargetCount;
        try 
        {
            ColorImage rawImage = null;
            try
            {
                rawImage = camera.getImage();
            } 
            catch (AxisCameraException ex) 
            {} 
            catch (NIVisionException ex) 
            {}
            rawImage.write("/rawImage.jpg");
            ColorImage image = new RGBImage("/rawImage.jpg");
            BinaryImage thresholdImage = image.thresholdHSV(80, 153, 100, 255, 45, 255); //keep only blue objects
            BinaryImage filteredImage = thresholdImage.particleFilter(cc); //filter out small particles
            
            Scores scores[] = new Scores[filteredImage.getNumberParticles()];
            horizontalTargetCount = verticalTargetCount = 0;
            
            if (filteredImage.getNumberParticles() > 0) 
            {
                for (int i = 0; i < MAX_PARTICLES && i < filteredImage.getNumberParticles(); i++) 
                {
                    ParticleAnalysisReport report = filteredImage.getParticleAnalysisReport(i);
                    scores[i] = new Scores();
                    scores[i].rectangularity = scoreRectangularity(report);
                    scores[i].aspectRatioVertical = scoreAspectRatio(filteredImage, report, i, true);
                    scores[i].aspectRatioHorizontal = scoreAspectRatio(filteredImage, report, i, false);
                    if (scoreCompare(scores[i], false)) 
                    {
                        horizontalTargets[horizontalTargetCount++] = i;
                    } 
                    else if (scoreCompare(scores[i], true)) 
                    {
                        verticalTargets[verticalTargetCount++] = i;
                    } 
                    else 
                    {}
                    return false;
                }
                target.totalScore = target.leftScore = target.rightScore = target.tapeWidthScore = target.verticalScore = 0;
                target.verticalIndex = verticalTargets[0];
                for (int i = 0; i < verticalTargetCount; i++) 
                {
                    ParticleAnalysisReport verticalReport = filteredImage.getParticleAnalysisReport(verticalTargets[i]);
                    for (int j = 0; j < horizontalTargetCount; j++) 
                    {
                        ParticleAnalysisReport horizontalReport = filteredImage.getParticleAnalysisReport(horizontalTargets[j]);
                        double horizWidth, horizHeight, vertWidth, leftScore, rightScore, tapeWidthScore, verticalScore, total;
                        horizWidth = NIVision.MeasureParticle(filteredImage.image, horizontalTargets[j], false, NIVision.MeasurementType.IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE);
                        vertWidth = NIVision.MeasureParticle(filteredImage.image, verticalTargets[i], false, NIVision.MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);
                        horizHeight = NIVision.MeasureParticle(filteredImage.image, horizontalTargets[j], false, NIVision.MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);
                        leftScore = ratioToScore(1.2 * (verticalReport.boundingRectLeft - horizontalReport.center_mass_x) / horizWidth);
                        rightScore = ratioToScore(1.2 * (horizontalReport.center_mass_x - verticalReport.boundingRectLeft - verticalReport.boundingRectWidth) / horizWidth);
                        tapeWidthScore = ratioToScore(vertWidth / horizHeight);
                        verticalScore = ratioToScore(1 - (verticalReport.boundingRectTop - horizontalReport.center_mass_y) / (4 * horizHeight));
                        total = leftScore > rightScore ? leftScore : rightScore;
                        total += tapeWidthScore + verticalScore;
                        if (total > target.totalScore) 
                        {
                            target.horizontalIndex = horizontalTargets[j];
                            target.verticalIndex = verticalTargets[i];
                            target.totalScore = total;
                            target.leftScore = leftScore;
                            target.rightScore = rightScore;
                            target.tapeWidthScore = tapeWidthScore;
                            target.verticalScore = verticalScore;
                        }
                        return false;
                    }
                    target.Hot = hotOrNot(target);
                    return false;
                }
                
                if (verticalTargetCount > 0)
                {
                    if (target.Hot) //if the hot target has been located
                    {
                        hot = true; //it's hot!
                    }
                    else 
                    {
                        hot = false; //not hot
                    }
                }
            }
                
            //MUST FREE IMAGES IN ORDER TO FREE SPACE, OTHERWISE THEY STACK
            rawImage.free();
            image.free();
            filteredImage.free();
            filteredImage.free();

            }
            catch (NIVisionException ex) 
            {}
            return true;
    }
    
    /**
     * @param image The image to use for measuring the particle estimated rectangle
     * @param report The Particle Analysis Report for the particle, used for the width, height and particle number
     * @param particleNumber 
     * @param vertical 
     * @return aspectRatio The aspect ratio score (0-100)
     * @throws NIVisionException
     */
    public double scoreAspectRatio(BinaryImage image, ParticleAnalysisReport report, int particleNumber, boolean vertical) throws NIVisionException
    {
        double rectLong, rectShort, aspectRatio, idealAspectRatio;

        rectLong = NIVision.MeasureParticle(image.image, particleNumber, false, NIVision.MeasurementType.IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE);
        rectShort = NIVision.MeasureParticle(image.image, particleNumber, false, NIVision.MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);
        idealAspectRatio = vertical ? (4.0 / 32) : (23.5 / 4);	//Vertical reflector 4" wide x 32" tall, horizontal 23.5" wide x 4" tall

        //Divide width by height to measure aspect ratio
        if (report.boundingRectWidth > report.boundingRectHeight) 
        {
            //particle is wider than it is tall, divide long by short
            aspectRatio = ratioToScore((rectLong / rectShort) / idealAspectRatio);
        } 
        else 
        {
            //particle is taller than it is wide, divide short by long
            aspectRatio = ratioToScore((rectShort / rectLong) / idealAspectRatio);
        }
        return aspectRatio;
    }
    
    /**
     *
     * @param scores The structure containing the scores to compare
     * @param vertical True if the particle should be treated as an outer targeting, false to treat it as a center target
     * 
     * @return True if the particle meets all limits, false otherwise
     */
    public boolean scoreCompare(Scores scores, boolean vertical) 
    {
        boolean isTarget = true;

        isTarget &= scores.rectangularity > RECTANGULARITY_LIMIT;
        if (vertical) 
        {
            isTarget &= scores.aspectRatioVertical > ASPECT_RATIO_LIMIT;
        } 
        else 
        {
            isTarget &= scores.aspectRatioHorizontal > ASPECT_RATIO_LIMIT;
        }

        return isTarget;
    }
    
    /**
     *
     * @param report The Particle Analysis Report for the particle to score
     * @return The rectangularity score (0 - 100)
     */
    public double scoreRectangularity (ParticleAnalysisReport report) 
    {
        if (report.boundingRectWidth * report.boundingRectHeight != 0) 
        {
            return 100 * report.particleArea / (report.boundingRectWidth * report.boundingRectHeight);
        } 
        else 
        {
            return 0;
        }
    }
    
    /**
     *
     * @param ratio 
     * @return
     */
    public double ratioToScore (double ratio) 
    {
        return (Math.max(0, Math.min(100 * (1 - Math.abs(1 - ratio)), 100)));
    }
    
    /**
     * This method is to determine the distance from the robot to the goal
     * @param image The image to use for measuring the particle estimated rectangle
     * @param report The Particle Analysis Report for the particle
     * @param particleNumber
     * @return The estimated distance to the target in Inches.
     * @throws NIVisionException 
     */
    public double computeDistance (BinaryImage image, ParticleAnalysisReport report, int particleNumber) throws NIVisionException 
    {
            double rectLong, height;
            int targetHeight;

            rectLong = NIVision.MeasureParticle(image.image, particleNumber, false, NIVision.MeasurementType.IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE);
            //using the smaller of the estimated rectangle long side and the bounding rectangle height results in better performance
            //on skewed rectangles
            height = Math.min(report.boundingRectHeight, rectLong);
            targetHeight = 32;

            return Y_IMAGE_RES * targetHeight / (height * 12 * 2 * Math.tan(VIEW_ANGLE*Math.PI/(180*2)));
    }
    
    /**
     *
     * @param target The target to be processed and tested
     * @return True if the target is hot, False if it is not
     */
    public boolean hotOrNot (TargetReport target) 
    {
        boolean isHot = true;

        isHot &= target.tapeWidthScore >= TAPE_WIDTH_LIMIT;
        isHot &= target.verticalScore >= VERTICAL_SCORE_LIMIT;
        isHot &= (target.leftScore > LR_SCORE_LIMIT) | (target.rightScore > LR_SCORE_LIMIT);

        return isHot;
    }
    /////////////////////END OF METHODS USED TO DETERMINE HOT GOAL/////////////////////////////////////////////
}
