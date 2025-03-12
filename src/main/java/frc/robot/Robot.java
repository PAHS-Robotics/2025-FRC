// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kMiddleAuto = "Middle Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final SparkMax rightLeader = new SparkMax(12, MotorType.kBrushed);
  private final SparkMax rightFollower = new SparkMax(11, MotorType.kBrushed);
  private final SparkMax leftLeader = new SparkMax(16, MotorType.kBrushed);
  private final SparkMax leftFollower = new SparkMax(13, MotorType.kBrushed);

  private final DifferentialDrive myDrive = new DifferentialDrive(leftLeader, rightLeader);

  private final SparkMax elevMotor = new SparkMax(14, MotorType.kBrushed);
  private final RelativeEncoder elevEncoder = elevMotor.getEncoder();

  private final SparkMax coralMotor = new SparkMax(17, MotorType.kBrushless);

  private final SparkMaxConfig driveConfig = new SparkMaxConfig();
  private final SparkMaxConfig rollerConfig = new SparkMaxConfig();

  private final Timer timer1 = new Timer();

  private final XboxController gamepad1 = new XboxController(0);
  private final XboxController gamepad2 = new XboxController(1);

  //elevator positions
  private static final double ELEV_BOTTOM = 0.0;
  private static final double ELEV_INTAKE = 10.0;
  private static final double ELEV_L2 = 20.0;
  
  //motor speeds
  private static final double ELEV_UP_SPEED = 0.4;
  private static final double ELEV_DOWN_SPEED = 0.25;

  //target positions
  private double elevatorTargetPosition = ELEV_BOTTOM;
  private boolean elevatorMoving = false;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    //CameraServer.startAutomaticCapture();

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Middle Auto", kMiddleAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    driveConfig.smartCurrentLimit(40); // Reduce from 60A to 40A
    driveConfig.voltageCompensation(12);

    driveConfig.follow(leftLeader);
    leftFollower.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveConfig.follow(rightLeader);
    rightFollower.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveConfig.disableFollowerMode();
    driveConfig.inverted(true);
    rightLeader.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveConfig.openLoopRampRate(0.5); // Add a 0.5 second ramp rate
    
    //flip left motor to reverse
    driveConfig.inverted(false);
    leftLeader.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
    rollerConfig.smartCurrentLimit(30);
    rollerConfig.voltageCompensation(10);
    elevMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    timer1.start();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    timer1.restart();
    timer1.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kMiddleAuto:
        if(timer1.get() < 3.5){
          myDrive.arcadeDrive(-0.5, 0);
        }
        else if (timer1.get() < 5){
          myDrive.arcadeDrive(0, 0);
          elevMotor.set(0);
        }
        else if (timer1.get() < 7){
          elevMotor.set(0);
          coralMotor.set(-0.1);
        }
        else {
          coralMotor.set(0);
        }
        break;
      case kDefaultAuto:
        System.out.println("defalt auto");
        myDrive.arcadeDrive(-0.3, 0);
        break;
      default:
        if (timer1.get() < 3){
          myDrive.arcadeDrive(-0.3, 0);
        }
        else {
          myDrive.arcadeDrive(0, 0);
        }
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //arcade drive
    double throttle = gamepad1.getLeftY();
    double rotation = gamepad1.getRightX() * 0.8;

    myDrive.arcadeDrive(throttle, rotation);

    /*elevator motor
    if (gamepad2.getLeftTriggerAxis() == 1){
      double leftStickY = gamepad2.getLeftY();
      elevMotor.set(-leftStickY * 0.8);
    }
    else if (gamepad2.getAButton()){
      System.out.println("hello world");
    }*/

    //elevator motor
    double elevatorCurrentPosition = elevEncoder.getPosition();
    
    if (gamepad2.getLeftTriggerAxis() == 1) {
        double leftStickY = gamepad2.getLeftY();
        elevMotor.set(-leftStickY * 0.8);
        elevatorMoving = false;
    } 
    else if (gamepad2.getAButtonPressed()) {
        elevatorTargetPosition = ELEV_BOTTOM;
        elevatorMoving = true;
    }
    else if (gamepad2.getBButtonPressed()) {
        elevatorTargetPosition = ELEV_INTAKE;
        elevatorMoving = true;
    }
    else if (gamepad2.getYButtonPressed()) {
        elevatorTargetPosition = ELEV_L2;
        elevatorMoving = true;
    }
    
    if (elevatorMoving) {
        elevatorCurrentPosition = elevEncoder.getPosition();
        
        double positionError = elevatorTargetPosition - elevatorCurrentPosition;
        double deadband = 0.5;
        
        if (Math.abs(positionError) < deadband) {
            elevMotor.set(0);
            elevatorMoving = false;
        }
        else if (positionError > 0) {
            elevMotor.set(-Math.abs(ELEV_UP_SPEED));
        }
        else {
            elevMotor.set(Math.abs(ELEV_DOWN_SPEED));
        }
    }
    else if (gamepad2.getLeftTriggerAxis() != 1) {
        elevMotor.set(0);
    }

    //coral motor
    double rightStickY = gamepad2.getRightY();
    coralMotor.set(rightStickY);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
