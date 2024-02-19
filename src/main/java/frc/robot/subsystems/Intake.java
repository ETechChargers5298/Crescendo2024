package frc.robot.subsystems;


import frc.robot.Ports;
import frc.robot.Constants.MechConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import com.revrobotics.ColorSensorV3;
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
    public static boolean isNoteFound = false;
    private final Color kOrangeTarget = new Color(0.5, 0.4, 0.1);


    private Intake() {
        intakeMotor = new CANSparkMax(Ports.INTAKE_MOTOR_PORT, MotorType.kBrushless);
        noteFinder = new ColorSensorV3(i2cPort);
    }
    
    public static Intake getInstance(){
        if(instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    //Elijah did this
    public void eat(){
        intakeMotor.set(-MechConstants.INTAKE_SPEED);
    }

    //Elijah did this
    public void spitt(){
        intakeMotor.set(MechConstants.INTAKE_SPEED);
    }

    //Elijah did this
    public void stop(){
        intakeMotor.set(0);
    }

    public Color getColor() {
        Color detectedColor = noteFinder.getColor();
        return detectedColor;
    }

    public boolean checkNoteFound(){
        Color detectedColor = noteFinder.getColor();
        if (detectedColor.red > 0.4){
            isNoteFound = true;
        } else{
            isNoteFound = false;
        }
        return isNoteFound;
    }
  
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
        Color detectedColor = noteFinder.getColor();
        //ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
        
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
        String colorString = match.color.toString();

        this.checkNoteFound();
        
        SmartDashboard.putNumber("R", detectedColor.red);
        SmartDashboard.putNumber("G", detectedColor.green);
        SmartDashboard.putNumber("B", detectedColor.blue);
        SmartDashboard.putNumber("Confidence", match.confidence);
        SmartDashboard.putString("Detected Color ", colorString);
        SmartDashboard.putBoolean("Have Note", isNoteFound);
    }

}
