// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  DigitalInput lowSwitch = new DigitalInput(0);
  DigitalInput portSwitch = new DigitalInput(1);
  DigitalInput highSwitch = new DigitalInput(2);


  double steering;
  double power;
  double throttle;

  enum ArmPos {
    LOW, MEDIUM, HIGH, IDLE,
  }

  ArmPos armPos;
  ArmPos desiredArmPos;


  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    armPos = ArmPos.HIGH;
    //desiredArmPos = ArmPos.IDLE;
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  VictorSP leftDrive = new VictorSP(0); // two controllers off pwm splitter
  VictorSP rightDrive = new VictorSP(1);

  VictorSP arm = new VictorSP(4);
  VictorSP intake = new VictorSP(3);

  Joystick stick = new Joystick(0);

  Compressor comp = new Compressor(0);

  DoubleSolenoid intakeSolenoid = new DoubleSolenoid(2, 3);
 // DoubleSolenoid controlPanelSolenoid = new DoubleSolenoid(0, 1);
  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
    case kCustomAuto:
      // Put custom auto code here
      break;
    case kDefaultAuto:
    default:
      // Put default auto code here
      break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    comp.start();
    intakeSolenoid.set(Value.kForward);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    System.out.println("Lower switch " + !lowSwitch.get());
    System.out.println("Middle switch " + !portSwitch.get());
    System.out.println("Upper switch " + !highSwitch.get());

    steering = stick.getX();
    power = stick.getY();
    throttle = (((stick.getThrottle() * -1) + 1) / 2);
    //armStateMachine();
    deadZoneCorrection();

    leftDrive.set(-(steering + power) * throttle);
    rightDrive.set(-(steering - power) * throttle);

    if (stick.getRawButton(11)) {
      System.out.println("DOWN!!!");
      armPos = ArmPos.LOW;
      armLow();
    } else if(stick.getRawButton(9)) {
      armPort();
    } else if(stick.getRawButton(7)) {
      armPos = ArmPos.HIGH;
      armHigh();
    } else {
      armIdle();
    }
    System.out.println(armPos);

    if(stick.getTrigger()) {
      intake.set(-1);
      intakeSolenoid.set(Value.kReverse);
    } else if(stick.getRawButton(2)) {
      intake.set(1);
    } else {
      intake.set(0);
      intakeSolenoid.set(Value.kForward);

    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  void deadZoneCorrection() {
    if (steering > -0.5 && steering < 0.5) {
      steering = 0;
    } else if (power > -0.5 && power < 0.5) {
      power = 0;
    }
  }

  void armLow() {
    if((lowSwitch.get())) {
      arm.set(1);
      armPos = ArmPos.LOW;
    } else {
      armIdle();
   }
  }
  void armPort() {
    if((portSwitch.get() && highSwitch.get())) {
      arm.set(-1);
    } else {
      armIdle();
    }

  }

  void armHigh() {
    if((highSwitch.get())) {
      arm.set(-1);
    } else {
      armIdle();
    }
  }

  void armStateMachine() {
    switch(desiredArmPos) {
      case LOW:
        armLow();
      break;
      case MEDIUM:
        armPort();
      break;
      case HIGH:
        armHigh();
      break;
      default:
      break;
    }
  }

  void armIdle() {
    arm.set(0);
  }
}
