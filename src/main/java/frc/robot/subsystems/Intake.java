package frc.robot.subsystems;


import frc.robot.Ports;
import frc.robot.Constants.MechConstants;
import frc.robot.subsystems.LEDStrip.SubsystemsPriority;
import frc.robot.utils.LEDColors;

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
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;


public class Intake extends SubsystemBase {
    
    private CANSparkMax intakeMotor;
    private static Intake instance;
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private ColorSensorV3 noteFinder;
    private final ColorMatch colorMatcher = new ColorMatch();
    public static boolean isNoteFound = false;
    private final Color kOrangeTarget = new Color(0.5, 0.4, 0.1);
    private Timer timer;
    private SerialPort arduino;


    private Intake() {
        intakeMotor = new CANSparkMax(Ports.INTAKE_MOTOR_PORT, MotorType.kBrushless);
        noteFinder = new ColorSensorV3(i2cPort);
        arduinoInit();

    }
    
    public void arduinoInit(){
        
        try {
            arduino = new SerialPort(9600, SerialPort.Port.kUSB1);
            System.out.println(("Connected on kUBS01!"));
        } catch (Exception e1) {
            System.out.println("Failed to connect on kUSB1");
        }
        
        timer = new Timer();
        timer.start();
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

    public int getDistance() {
        return noteFinder.getProximity();
    }

    public boolean checkNoteFound(){
        Color detectedColor = noteFinder.getColor();
        if (detectedColor.red > 0.4 || getDistance() > 137){
            isNoteFound = true;
        } else{
            isNoteFound = false;
        }
        return isNoteFound;
    }
  
    public void arduinoUpdate(){

        double ARDUINO_UPDATE_GAP = 0.5;
        if(timer.get() > ARDUINO_UPDATE_GAP) {
            System.out.println("Wrote to Arduino");
            arduino.write(new byte[] {0x12}, 1);
            timer.reset();
        }

        if (arduino.getBytesReceived() > 0) {
            String currentString = arduino.readString();

            if (currentString.equals("0"))

            System.out.print(currentString);
        }
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        arduinoUpdate();

        // Color detectedColor = noteFinder.getColor();
        // //ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
        
        // ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
        // String colorString = match.color.toString();

        // this.checkNoteFound();
        
        // SmartDashboard.putNumber("R", detectedColor.red);
        // SmartDashboard.putNumber("G", detectedColor.green);
        // SmartDashboard.putNumber("B", detectedColor.blue);
        // SmartDashboard.putNumber("Confidence", match.confidence);
        // SmartDashboard.putString("Detected Color ", colorString);
        SmartDashboard.putBoolean("Have Note", isNoteFound);
 

        if(checkNoteFound()) {
            LEDStrip.request(SubsystemsPriority.NOTE, LEDStrip.HAVE_NOTE);
        } 

        SmartDashboard.putNumber("Note Distance", getDistance());
    }

}
