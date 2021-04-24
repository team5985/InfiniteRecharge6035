// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.ControlPanel.ControlPanelState;
import frc.util.ColourSensor;
import frc.util.UltrasonicI2C;

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

  UltrasonicI2C usi2cl;
  UltrasonicI2C usi2cr;

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
    
    I2C.Port i2cp = I2C.Port.kOnboard;
    I2C usLinkl = new I2C(i2cp, 0x14);
    I2C usLinkr = new I2C(i2cp, 0x13);
    usi2cl = new UltrasonicI2C(usLinkl);
    usi2cr = new UltrasonicI2C(usLinkr);
  }

  VictorSP leftDrive = new VictorSP(0); // two controllers off pwm splitter
  VictorSP rightDrive = new VictorSP(1);
  int autoSeconds = 0;

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
    UltrasonicI2C.usResults resultsr = usi2cr.getResults();
    double resr = resultsr.getResult();
    SmartDashboard.putNumber("Distance right", resr);
    UltrasonicI2C.usResults resultsl = usi2cl.getResults();
    double resl = resultsl.getResult();
    SmartDashboard.putNumber("Distance left", resl);
  }

  /**
   * Our own function that we call at the *start* of each periodic scan.
   * In here we monitor and update our sensors.
   * 
   * For sending data to teh SmartDashboard you should probably put it in
   * robotPeriodic() with is run at the *end* of each scan.
   */
  public void updatePeriodic()
  {
    // Scan the sensors and process/count/whatever the information coming in.
    ColourSensor.getInstance().update();
    usi2cl.update();
    usi2cr.update();
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
    updatePeriodic();
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
   
    if(autoSeconds<= 80){
    leftDrive.set(0.5);
    rightDrive.set(-0.5);
    }
    autoSeconds++;
    if(autoSeconds > 80){
      leftDrive.set(0);
      rightDrive.set(0);
    }

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    comp.start();
    intakeSolenoid.set(Value.kForward);
  }

  private boolean lastSpinnyPadButton = false;

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    updatePeriodic();
    cPanel.update();

    if (stick.getPOV() == 45)
    {
        usWallFollower(false, true);
    }
    else if (stick.getPOV() == 135)
    {
        usWallFollower(true, true);
    }
    else if (stick.getPOV() == 225)
    {
        usWallFollower(true, false);
    }
    else if (stick.getPOV() == 315)
    {
        usWallFollower(false, false);
    }
    else
    {
        throttle = (-1 * stick.getThrottle() + 1) / 2;
        steering = stick.getX() * throttle;
        power = -1 * stick.getY() * throttle;

        if (steering > -kJoystickDeadband && steering < kJoystickDeadband) {
          steering = 0;
        }
        if (power > -kJoystickDeadband && power < kJoystickDeadband) {
          power = 0;
        }
        if (cPanel.getRotating() && (power == 0))
        {
            power = -0.1;
        }
        steerPriority(power + steering, power - steering);
    }

    // steering = -stick.getX();
    // power = stick.getY();
    // throttle = (((stick.getThrottle() * -1) + 1) / 2);
    // deadZoneCorrection();

    // leftDrive.set(-(steering + power) * throttle);
    // rightDrive.set(-(steering - power) * throttle);

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
    // System.out.println(armPos);

    if (pad.getRawButton(1) && !lastSpinnyPadButton)
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
    lastSpinnyPadButton = pad.getRawButton(1); 


    if(cPanel.getExtended() == true)
    {
      if (stick.getRawButton(1))
      {
        cPanel.setDesiredState(ControlPanelState.ROTATION_CONTROL);
      }
      else if(stick.getRawButton(2))
      {
        cPanel.setDesiredState((ControlPanelState.POSITION_CONTROL));

      }
      else if(stick.getRawButton(3)){
        cPanel.setDesiredState((ControlPanelState.MANUAL_ANTICLOCKWISE));
      }
      else if(stick.getRawButton(4))
      {
        cPanel.setDesiredState((ControlPanelState.MANUAL_CLOCKWISE));
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

  private double lastError = 0;
  private double outSpeed = 0;

  private void usWallFollower(boolean reverse, boolean right)
  {        
      UltrasonicI2C.usResults resultsr = usi2cr.getResults();
      UltrasonicI2C.usResults resultsl = usi2cl.getResults();

      double aimPos = 300; // how far away from the wall we want to be in mm
      double pgain = 0.00025; // how fast we correct ourselves
      double dgain = 0.005; // change in gain
      double speed = 1;
      double leftPower;
      double rightPower;
      double accRate = 0.08;

      // Tuning for 6035 main robot.
      speed = 1.0;
      aimPos = 350;
      pgain = 0.0005;
      dgain = 0.02
      ;

      double power = speed;
      if (reverse)
      {
          power = -speed;
      }

      outSpeed = outSpeed + Math.min( Math.max((power - outSpeed), -accRate), accRate);

      double dirPGain = pgain;
      double dirDGain = dgain;
      if (outSpeed < 0)
      {
          dirPGain = -dirPGain;
          dirDGain = -dirDGain;
      }
      double error = 0;
      if (right)
      {
          error = resultsr.getResult() - aimPos; // how far off from aimPos we are
      }
      else   
      {
          error = aimPos - resultsl.getResult(); // how far off from aimPos we are
      }
      double delta = 0;
      if ((right && resultsr.getNew()) || (!right && resultsl.getNew()))
      {
          delta = error - lastError; // the change between error and lastError
          lastError = error;
      }
      steering = (error * dirPGain) + (delta * dirDGain);
      double pOutput = error * dirPGain;
      double dOutput = delta * dirDGain;
      SmartDashboard.putNumber("pOutput", pOutput);
      SmartDashboard.putNumber("dOutput", dOutput);
      SmartDashboard.putNumber("Error", error);
      leftPower = outSpeed + steering;
      rightPower = outSpeed - steering;
      steerPriority(leftPower, rightPower);
  }

  private void steerPriority(double left, double right)
  {
      if (left - right > 2)
      {
          left = 1;
          right = -1;
      }
      else if (right - left > 2)
      {
          left = -1;
          right = 1;
      }
      else if (Math.max(right, left) > 1)
      {
          left = left - (Math.max(right,left) - 1);
          right = right - (Math.max(right,left) - 1);
      }
      else if (Math.min(right, left) < -1)
      {
          left = left - (Math.min(right,left) + 1);
          right = right - (Math.min(right,left) + 1);
      }
      SmartDashboard.putNumber("leftPower", left);
      SmartDashboard.putNumber("rightPower", left);
      SmartDashboard.putNumber("SteerLeft", left-right);
      leftDrive.set(left);
      rightDrive.set(-right);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    updatePeriodic();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    updatePeriodic();
  }

  // void deadZoneCorrection() {
  //   if (steering > -kJoystickDeadband && steering < kJoystickDeadband) {
  //     steering = 0;
  //   }
  //   if (power > -kJoystickDeadband && power < kJoystickDeadband) {
  //     power = 0;
  //   }
  // }

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
