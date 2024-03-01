package frc.robot.subsystems;

//Arduino Code to get Color Sensor values:
//https://app.arduino.cc/sketches/000e67ae-1a72-4390-8ecf-1de067ed37af

import frc.robot.Ports;
import frc.robot.Constants.MechConstants;
import frc.robot.subsystems.LEDStrip.SubsystemPriority;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import com.revrobotics.ColorSensorV3;
// import com.revrobotics.ColorMatchResult;
// import com.revrobotics.ColorMatch;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;


public class Intake extends SubsystemBase {
    
    private CANSparkMax intakeMotor;
    private static Intake instance;
    // private final I2C.Port i2cPort = I2C.Port.kOnboard;
    // private ColorSensorV3 noteFinder;
    //private final ColorMatch colorMatcher = new ColorMatch();
    //private final Color kOrangeTarget = new Color(0.5, 0.4, 0.1);

    public static boolean isNoteFound = false;
    public static int red = -1;
    public static int green = -1;
    public static int blue = -1;
    private Timer timer;
    private SerialPort arduino;


    private Intake() {
        intakeMotor = new CANSparkMax(Ports.INTAKE_MOTOR_PORT, MotorType.kBrushless);
        // noteFinder = new ColorSensorV3(i2cPort);
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


    //------------------COLOR SENSOR METHODS----------------//
    // public Color getColor() {
    //     Color detectedColor = noteFinder.getColor();
    //     return detectedColor;
    // }

    // public int getDistance() {
    //     return noteFinder.getProximity();
    // }

    public boolean checkNoteFound(){
        
        //update the color rgb
        //Color detectedColor = noteFinder.getColor();

        //determine if a note is found
        // if (detectedColor.red > 0.4 || getDistance() > 137){
        //     isNoteFound = true;
        // } else{
        //     isNoteFound = false;
        // }
        return isNoteFound;
    }
  
    public void arduinoUpdate(){

        //See for color parsing:
        //https://replit.com/@misterbianchi/RGB-Int-Values-from-Serial#src/main/java/Main.java

        double ARDUINO_UPDATE_GAP = 0.05;
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
        // ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
        // String colorString = match.color.toString();
        // this.checkNoteFound();
        // SmartDashboard.putNumber("R", detectedColor.red);
        // SmartDashboard.putNumber("G", detectedColor.green);
        // SmartDashboard.putNumber("B", detectedColor.blue);
        //SmartDashboard.putNumber("Note Distance", getDistance());

        SmartDashboard.putBoolean("Have Note", isNoteFound);
        SmartDashboard.putNumber("R", this.red);
        SmartDashboard.putNumber("G", this.green);
        SmartDashboard.putNumber("B", this.blue);
 
        if(checkNoteFound()) {
            LEDStrip.request(SubsystemPriority.NOTE, LEDStrip.HAVE_NOTE);
        } 


    }

}
