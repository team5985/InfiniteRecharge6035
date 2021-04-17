// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.ControlPanel.ControlPanelState;
import frc.util.ColourSensor;

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
  public final static double kJoystickDeadband = 0.2;

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
    CameraServer.getInstance().startAutomaticCapture(0);
    cPanel = new ControlPanel(spinny, spinnySolenoid);
    armPos = ArmPos.HIGH;
    //desiredArmPos = ArmPos.IDLE;
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  VictorSP leftDrive = new VictorSP(0); // two controllers off pwm splitter
  VictorSP rightDrive = new VictorSP(1);

  VictorSP arm = new VictorSP(7);
  VictorSP intake = new VictorSP(5);
  VictorSP spinny = new VictorSP(3);

  Joystick stick = new Joystick(0);
  Joystick pad = new Joystick(1);

  Compressor comp = new Compressor(0);

  DoubleSolenoid intakeSolenoid = new DoubleSolenoid(2, 3);
  DoubleSolenoid spinnySolenoid = new DoubleSolenoid(0, 1);

  ControlPanel cPanel;

  public static final int ARM_POS_NONE = 0;
  public static final int ARM_POS_HIGH = 1;
  public static final int ARM_POS_PORT = 2;
  public static final int ARM_POS_LOW = 3;
  int armCommand = ARM_POS_NONE;

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
    ColourSensor.getInstance().update();
    cPanel.update();

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

  private boolean lastSpinnyButton = false;

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    ColourSensor.getInstance().update();
    cPanel.update();

    steering = -stick.getX();
    power = stick.getY();
    throttle = (((stick.getThrottle() * -1) + 1) / 2);
    deadZoneCorrection();

    leftDrive.set(-(steering + power) * throttle);
    rightDrive.set(-(steering - power) * throttle);

    boolean padMove = false;
    if (pad.getPOV() >= 0)
    {
      padMove = true;
    }

    if (( stick.getRawButton(11) || pad.getRawButton(2) )) {
      armPos = ArmPos.LOW;
      armLow();
      if (padMove)
      {
        armCommand = ARM_POS_LOW;
      }
      else
      {
        armCommand = ARM_POS_NONE;
      }
    } else if((stick.getRawButton(9) || pad.getRawButton(3) )) {
      armPort();
      if (padMove)
      {
        armCommand = ARM_POS_PORT;
      }
      else
      {
        armCommand = ARM_POS_NONE;
      }
    } else if((stick.getRawButton(7) || pad.getRawButton(4) )) {
      armPos = ArmPos.HIGH;
      armHigh();
      if (padMove)
      {
        armCommand = ARM_POS_HIGH;
      }
      else
      {
        armCommand = ARM_POS_NONE;
      }
    } else if (armCommand != ARM_POS_NONE) {
      if (armCommand == ARM_POS_LOW)
      {
        armLow();
        if(lowSwitch.get())
        {
          armCommand = ARM_POS_NONE;
        }
      } else if (armCommand == ARM_POS_PORT)
      {
        armPort();
        if((!portSwitch.get() || highSwitch.get()))
        {
          armCommand = ARM_POS_NONE;
        }
      } else if (armCommand == ARM_POS_HIGH)
      {
        armHigh();
        if(highSwitch.get()) {
          armCommand = ARM_POS_NONE;
        }
      } else {
        armCommand = ARM_POS_NONE;
        armIdle();
      }
    } else {
      armIdle();
    }
    System.out.println(armPos);

    if (stick.getRawButton(12) && !lastSpinnyButton)
    {
      if (cPanel.getExtended())
      {
        cPanel.setDesiredState(ControlPanelState.RETRACTED);
      }
      else
      {
        cPanel.setDesiredState(ControlPanelState.EXTENDED);
      }
    }
    lastSpinnyButton = stick.getRawButton(12);

    if(cPanel.getExtended() == true)
    {
      if (stick.getRawButton(1))
      {
        cPanel.setDesiredState(ControlPanelState.MANUAL_CLOCKWISE);
      }
      else
      {
        cPanel.setDesiredState(ControlPanelState.EXTENDED);
      }
      intake.set(0);
      intakeSolenoid.set(Value.kForward);
    }
    else
    {
      if((stick.getTrigger() || pad.getRawButton(7))) {
        // Intake balls
        //  Solenoid retracted
        //  motor intaking
        intake.set(-1);
        intakeSolenoid.set(Value.kReverse);
      } else if(( stick.getRawButton(2) || pad.getRawButton(8) )) {
        // Eject balls
        //  Solenoid extended
        //  motor outputting
        intake.set(1);
      } else if(pad.getRawButton(5)) {
        // Eject jammed ball only
        //  Solenoid retracted
        //  motor outputting
        intake.set(1);
      } else {
        intake.set(0);
        intakeSolenoid.set(Value.kForward);
      }
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
    if (steering > -kJoystickDeadband && steering < kJoystickDeadband) {
      steering = 0;
    }
    if (power > -kJoystickDeadband && power < kJoystickDeadband) {
      power = 0;
    }
  }

  void armLow() {
    if((!lowSwitch.get())) {
      arm.set(1);
      armPos = ArmPos.LOW;
    } else {
      armIdle();
   }
  }
  void armPort() {
    if((portSwitch.get() && !highSwitch.get())) {
      arm.set(-1);
    } else {
      armIdle();
    }

  }

  void armHigh() {
    if((!highSwitch.get())) {
      arm.set(-1);
    } else {
      armIdle();
    }
  }

  void armIdle() {
    arm.set(0);
  }
}
