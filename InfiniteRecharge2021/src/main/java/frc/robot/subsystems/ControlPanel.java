package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.Constants;
import frc.util.ColourSensor;

//public class ControlPanel extends Subsystem
public class ControlPanel
{
    /**
     * The state we are currently in
     */
    private ControlPanelState currentState;

    /**
     * The state we are going to be in.
     */
    private ControlPanelState desiredState;

    private int SecondarySpinCheck;
    private VictorSP spinMotor;
    private DoubleSolenoid spinSolenoid;

    // Prohibit default constructor from being used.
    @SuppressWarnings("unused")
    private ControlPanel(){}

    public ControlPanel(VictorSP aSpinMotor, DoubleSolenoid aSpinSolenoid){
        spinMotor = aSpinMotor;
        spinSolenoid = aSpinSolenoid;
        desiredState = ControlPanelState.RETRACTED;
        SecondarySpinCheck = 0;
    }

    /**
     * Run every 20ms, executes all functionality
     */
    public void update()
    {
        //This boolean is declared as true every time we enter a new state, and can be used
        // to differentiate between the first loop of the state from the others.
        boolean newState = false;    
        if (currentState != desiredState)
        {
            newState = true;
            currentState = desiredState;
        }

        //state machine
        switch(currentState) {
            //What to do if we are EXTENDING our arm, or we are EXTENDED
            case EXTENDED:
            setSpinnerSpeed(0);
            extendSpinner();
            break;

            //What to do if we are RETRACTING our arm, or we are RETRACTED
            case RETRACTED:
            setSpinnerSpeed(0);
            retractSpinner();
            break;

            // What to do during ROTATIONAL CONTROL. This is the end of stage 2
            // in the game where we need to rotate the wheel between 3 and 5
            // revolutions.
            case ROTATION_CONTROL:
            extendSpinner();
            if(newState == true){
                ColourSensor.getInstance().resetColourChanges();
            }
            double controlPanelRotations = ColourSensor.getInstance().getControlPanelRotations();
            if(controlPanelRotations < Constants.kRotationalControlTargetRotations)
            {   
                if(Constants.kRotationalControlTargetRotations - controlPanelRotations < 2)
                {
                    setSpinnerSpeed(Constants.kControlPanelPoisitionControlSpeed);
                }
                else
                {
                    setSpinnerSpeed(Constants.kRotationalControlSpeed);
                }
            }
            else
            {
                setSpinnerSpeed(0);
                // Do not auto retract the spinny on the 6035 robot or else you will hit the colour sensor on the way down!
                //setDesiredState(ControlPanelState.RETRACTED);
            }   
            break;

            // What to do during POSITIONAL CONTROL. This is where we need to turn
            // the wheel to a particular colour.
            case POSITION_CONTROL:
            extendSpinner();
            if(newState == true){
                SecondarySpinCheck = 0;
            }     
            if (ColourSensor.getInstance().getFmsColour() == Constants.kControlPanelColourInvalid)
            {
                // Do not auto retract the spinny on the 6035 robot or else you will hit the colour sensor on the way down!
                //setDesiredState(ControlPanelState.RETRACTED);
                setDesiredState(ControlPanelState.EXTENDED);
            }
            else
            {
                int FMSColour = ColourSensor.getInstance().getFmsColour();
                int currentColour = ColourSensor.getInstance().getColour();
                // Calculate target colour - two offset from the colour the robot will see.
                int TargetColour = (FMSColour +2) % 4;
                if(currentColour != TargetColour)
                {
                    setSpinnerSpeed(Constants.kControlPanelPoisitionControlSpeed);
                    SecondarySpinCheck = 0;
                }
                else
                {
                    if(SecondarySpinCheck >= 3)
                    {
                       // Do not auto retract the spinny on the 6035 robot or else you will hit the colour sensor on the way down!
                        //setDesiredState(ControlPanelState.RETRACTED);
                        setSpinnerSpeed(0);
                        setDesiredState(ControlPanelState.EXTENDED);
                    }
                    else
                    {
                        setSpinnerSpeed(Constants.kControlPanelPoisitionControlSpeed);
                    }
                    SecondarySpinCheck++;    
                }
            }
            break;

            //What to do when we are spinning MANUAL CLOCKWISE
            case MANUAL_CLOCKWISE:
            extendSpinner();
            setSpinnerSpeed(Constants.kControlPanelManualSpeed);
            break;

            //What to do when we are spinning MANUAL ANTICLOCKWISE
            case MANUAL_ANTICLOCKWISE:
            extendSpinner();
            setSpinnerSpeed(-Constants.kControlPanelManualSpeed);
            break;

            default:
            desiredState = ControlPanelState.RETRACTED;
            break;
        }
    }

    /**
     * This allows another class to change our control state.
     * 
     * @param newState The state to change to
     */
    public void setDesiredState(ControlPanelState newState)
    {
        desiredState = newState;
    }

    /**
     * Returns our current state
     * 
     * @return The current control state.
     */
    public ControlPanelState getCurrentState()
    {
        return currentState;
    }

    public boolean getExtended()
    {
        if(currentState == ControlPanelState.EXTENDED)
        {
            return true;
        }
        return false;
    }

    /**
     * An enum to list all of our states.
     */
    public enum ControlPanelState
    {
        EXTENDED,
        RETRACTED,
        ROTATION_CONTROL,
        POSITION_CONTROL,
        MANUAL_CLOCKWISE,
        MANUAL_ANTICLOCKWISE,
    }
    Solenoid controlPanelSolenoid;
    
    /**
     * Extends our spinner
     */
    private void extendSpinner()
    {
        spinSolenoid.set(Constants.kControlPanelExtendedState);
    }

    /**
    * Retracts spinner
    */
    private void retractSpinner()
    {
        spinSolenoid.set(Constants.kControlPanelRetractedState);
    }

    /**
     * Sets our spinner motor speed.
     */
    private void setSpinnerSpeed(double speed)
    {
        spinMotor.set(speed);
    }
}
