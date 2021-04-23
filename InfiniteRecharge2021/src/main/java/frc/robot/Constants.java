package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.util.Color;

public class Constants {

    public static final int kControlPanelMotor = 23;

	
	public static final int kPcmCanID = 50;
	public static final int kCompressorCanID = 51; //FIXME

	public static final int kSensorOffset = 2; //FIXME

	public static final boolean kControlPanelDoubleSolenoid = false;
	public static final int kControlPanelSolenoidAChannel = 7; //FIXME
	public static final int kControlPanelSolenoidBChannel = 6; //FIXME
	public static final double kRotationalControlTargetRotations = 3.5;
	//Depreciated if single solenoid
    public static final Value kControlPanelExtendedState = Value.kForward;
    public static final Value kControlPanelRetractedState = Value.kReverse;

	public static final double kControlPanelManualSpeed = 0.3; 
	public static final double kRotationalControlSpeed = 1.0;
	public static final double kControlPanelPoisitionControlSpeed = 0.5;

	public static final double kControlPanelTargetRotations = 4;
	public static final double kControlPanelHysteresis = 0.5;


	/**
     * Colour RED seen at the control panel.
     */
    public static final int kControlPanelColourRed = 0;

    /**
     * Colour GREEN seen at the control panel.
     */
    public static final int kControlPanelColourGreen = 1;

    /**
     * Colour BLUE seen at the control panel.
     */
    public static final int kControlPanelColourBlue = 2;

    /**
     * Colour YELLOW seen at the control panel.
     */
    public static final int kControlPanelColourYellow = 3;

    /**
     * The colour seen at the control panel  did not match any of the expected colours.
     */
    public static final int kControlPanelColourInvalid = -1;

    /**
     * The number of colour transitions per full rotation of the colour wheel.
     */
    public static final int kCPANEL_COLOURS_PER_ROTATION = 8;
  

	// Values from test with sensor LED turned on.
	

	//Official Colours

   /* public static final Color kLIT_CYAN = ColorMatch.makeColor(0.14,0.42,0.43);
    public static final Color kLIT_GREEN = ColorMatch.makeColor(0.19,0.53,0.27);
    public static final Color kLIT_RED = ColorMatch.makeColor(0.48,0.36,0.16);
	public static final Color kLIT_YELLOW = ColorMatch.makeColor(0.32,0.53,0.14); */

	//Our Colours
	public static final Color kLIT_CYAN = ColorMatch.makeColor(0.21,0.48,0.31);
    public static final Color kLIT_GREEN = ColorMatch.makeColor(0.23,0.58,0.19);
    public static final Color kLIT_RED = ColorMatch.makeColor(0.47,0.37,0.16);
	public static final Color kLIT_YELLOW = ColorMatch.makeColor(0.33,0.54,0.13);



	// Values from test with sensor LED turned off (ambient light).
	
	// Values with offical colours
    /*public static final Color kAMB_CYAN = ColorMatch.makeColor(0.14,0.39,0.45);
    public static final Color kAMB_GREEN = ColorMatch.makeColor(0.20,0.51,0.28);
    public static final Color kAMB_RED = ColorMatch.makeColor(0.59,0.29,0.12);
    public static final Color kAMB_YELLOW = ColorMatch.makeColor(0.38,0.49,0.13); */

	//Values with our colours
	public static final Color kAMB_CYAN = ColorMatch.makeColor(0.21,0.48,0.31);
    public static final Color kAMB_GREEN = ColorMatch.makeColor(0.23,0.58,0.19);
    public static final Color kAMB_RED = ColorMatch.makeColor(0.47,0.37,0.16);
	public static final Color kAMB_YELLOW = ColorMatch.makeColor(0.33,0.54,0.13);
	//Name Values for Colours
    public static final String kNAME_CYAN = "Cyan";
    public static final String kNAME_GREEN = "Green";
    public static final String kNAME_RED = "Red";
    public static final String kNAME_YELLOW = "Yellow";
    public static final String kNAME_UNKNOWN = "Unknown";
	


    // FMS values for colours
    public static final char kFMS_CYAN = 'B';
    public static final char kFMS_RED = 'R';
    public static final char kFMS_YELLOW = 'Y';
    public static final char kFMS_GREEN = 'G';
	public static Object getInstance() {
		return null;
	}

  
}
