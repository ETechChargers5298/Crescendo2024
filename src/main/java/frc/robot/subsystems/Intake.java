package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;


public class Intake extends SubsystemBase {
    
    private CANSparkMax intakeMotor;
    private static Intake instance;
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private ColorSensorV3 noteFinder;
    private final ColorMatch colorMatcher = new ColorMatch();
    private boolean isNoteFound = false;
    private final Color kOrangeTarget = new Color(1, 0.65, 0);
    


    private Intake() {
        intakeMotor = new CANSparkMax(Constants.MechConstants.INTAKE_MOTOR_PORT, MotorType.kBrushless);
        noteFinder = new ColorSensorV3(i2cPort);
    }
    
    public static Intake getInstance(){
        if(instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public Color getColor() {

        Color detectedColor = noteFinder.getColor();
        return detectedColor;
    }

    //Elijah did this
    public void eat(){
        intakeMotor.set(-Constants.MotorSpeeds.INTAKE_SPEED);
    }

    
    //Elijah did this
    public void spitt(){
        intakeMotor.set(Constants.MotorSpeeds.INTAKE_SPEED);
    }
 //Elijah did this
    public void stop(){
        intakeMotor.set(0);
    }

    public void noteFound(){
        colorMatcher.addColorMatch(kOrangeTarget);
        if(noteFinder.getColor() == kOrangeTarget){
            isNoteFound = true;
        }
        else{
            isNoteFound = false;
        }
    }

    @Override

    //Color Sensor Code
    public void periodic() {
        // This method will be called once per scheduler run
        
        Color detectedColor = noteFinder.getColor();
        //ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
        
        String colorString;
        
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

        if(match.color == kOrangeTarget){
            colorString = "Orange";
        }
        else{
            colorString = "Unknown Color";
        }

        // For detected.Color I haven't been able to find any ways to set it to Orange
        SmartDashboard.putNumber("Orange", detectedColor.red);
        SmartDashboard.putNumber("Confidence", match.confidence);
        SmartDashboard.putString("Detected Color ", colorString);
        SmartDashboard.putBoolean("Have Note", isNoteFound);
        

        
        //SmartDashboard.putNumber("Confidence", match.confidence);
        //SmartDashboard.putString("Detected Color", colorString);
    }

    // public boolean haveNote() {
  
    //  ColorMatchResult match = noteFinder.matchClosestColor(getColor());

    //     if (match.color == kOrangeTarget) {
    //     return true;
    //     }
    // }


//     /**
//    * Note: Any example colors should be calibrated as the user needs, these
//    * are here as a basic example.
//    */
//   private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
//   private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
//   private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
//   private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

//   @Override
//   public void robotInit() {
//     m_colorMatcher.addColorMatch(kBlueTarget);
//     m_colorMatcher.addColorMatch(kGreenTarget);
//     m_colorMatcher.addColorMatch(kRedTarget);
//     m_colorMatcher.addColorMatch(kYellowTarget);    
//   }

//   @Override
//   public void robotPeriodic() {
//     /**
//      * The method GetColor() returns a normalized color value from the sensor and can be
//      * useful if outputting the color to an RGB LED or similar. To
//      * read the raw color, use GetRawColor().
//      * 
//      * The color sensor works best when within a few inches from an object in
//      * well lit conditions (the built in LED is a big help here!). The farther
//      * an object is the more light from the surroundings will bleed into the 
//      * measurements and make it difficult to accurately determine its color.
//      */
//     Color detectedColor = m_colorSensor.getColor();

//     /**
//      * Run the color match algorithm on our detected color
//      */
//     String colorString;
//     ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

//     if (match.color == kBlueTarget) {
//       colorString = "Blue";
//     } else if (match.color == kRedTarget) {
//       colorString = "Red";
//     } else if (match.color == kGreenTarget) {
//       colorString = "Green";
//     } else if (match.color == kYellowTarget) {
//       colorString = "Yellow";
//     } else {
//       colorString = "Unknown";
//     }

//     /**
//      * Open Smart Dashboard or Shuffleboard to see the color detected by the 
//      * sensor.
//      */
//     SmartDashboard.putNumber("Red", detectedColor.red);
//     SmartDashboard.putNumber("Green", detectedColor.green);
//     SmartDashboard.putNumber("Blue", detectedColor.blue);
//     SmartDashboard.putNumber("Confidence", match.confidence);
//     SmartDashboard.putString("Detected Color", colorString);
//   }
// }

}