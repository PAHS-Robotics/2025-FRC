// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCenterCoral = "Center and Coral";
  private static final String kJustDrive = "Just Drive";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final SparkMax rightLeader = new SparkMax(12, MotorType.kBrushed);
  private final SparkMax rightFollower = new SparkMax(11, MotorType.kBrushed);
  private final SparkMax leftLeader = new SparkMax(16, MotorType.kBrushed);
  private final SparkMax leftFollower = new SparkMax(13, MotorType.kBrushed);

  private final DifferentialDrive myDrive = new DifferentialDrive(leftLeader, rightLeader);

  private final SparkMax elevMotor = new SparkMax(14, MotorType.kBrushed);
  private final Spark test = new Spark(0);

  private final SparkMaxConfig driveConfig = new SparkMaxConfig();
  private final SparkMaxConfig rollerConfig = new SparkMaxConfig();

  private final Timer timer1 = new Timer();

  private double ELEV_SPEED_VALUE = 0;
  private double DRIVE_SPEED = 1;

  private final XboxController gamepad1 = new XboxController(0);
  private final XboxController gamepad2 = new XboxController(1);

  final double BASE_SPEED = 0.4;
  final double MAX_TRIGGER_BOOST = 0.6;
  final double TURN_SENSITIVITY = 0.7;
  
  public Robot() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Center and Coral", kCenterCoral);
    m_chooser.addOption("Just Drive", kJustDrive);
    SmartDashboard.putData("Auto choices", m_chooser);

    driveConfig.smartCurrentLimit(60);
    driveConfig.voltageCompensation(12);

    driveConfig.follow(leftLeader);
    leftFollower.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveConfig.follow(rightLeader);
    rightFollower.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveConfig.disableFollowerMode();
    driveConfig.inverted(true);
    rightLeader.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Flip left motor to reverse
    driveConfig.inverted(false);
    leftLeader.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
    rollerConfig.smartCurrentLimit(60);
    rollerConfig.voltageCompensation(10);
    elevMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    timer1.start();
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    timer1.restart();
  }

  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      default:
      break;
    }
  }

  @Override
  public void teleopInit() {
    test.set(0);
  }

  @Override
  public void teleopPeriodic() {
    // Drive control with brake functionality
    double leftStickY = gamepad1.getLeftY();
    double rightStickX = gamepad1.getRightX();
    double rightTrigger = gamepad1.getRightTriggerAxis();
    double leftTrigger = gamepad1.getLeftTriggerAxis(); // Left trigger for braking

    double forwardSpeed = 0;
    if (Math.abs(leftStickY) > 0.1) {
        forwardSpeed = Math.signum(leftStickY) * (BASE_SPEED + (rightTrigger * MAX_TRIGGER_BOOST));
    }

    // Apply braking effect (reduces speed when left trigger is pressed)
    forwardSpeed *= (1 - (0.8 * leftTrigger));

    myDrive.arcadeDrive(forwardSpeed, rightStickX * TURN_SENSITIVITY);

    // Elevator motor with safe range
    double elevSpeed = -gamepad2.getLeftY();
    elevSpeed = Math.max(-0.7, Math.min(0.7, elevSpeed));
    elevMotor.set(elevSpeed);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}