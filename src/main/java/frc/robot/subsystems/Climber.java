package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.Constants.MechConstants;


public class Climber extends SubsystemBase{

    private CANSparkMax rightMotor;
    private CANSparkMax leftMotor;
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;
    private static Climber instance;

    private Climber(){
        this.rightMotor = new CANSparkMax(Ports.CLIMB_RIGHT_MOTOR_PORT, MotorType.kBrushless);
        this.leftMotor = new CANSparkMax(Ports.CLIMB_LEFT_MOTOR_PORT, MotorType.kBrushless);
        leftEncoder = leftMotor.getAlternateEncoder(MechConstants.ENCODER_TICKS);
        rightEncoder = rightMotor.getAlternateEncoder(MechConstants.ENCODER_TICKS);
    }

    public static Climber getInstance(){
        if(instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    public void climberReach(double speed){

        //Reach up only if not at the max height of right climber
        if(getRightHeight() >= MechConstants.MAX_CLIMB_RIGHT){
            rightMotor.set(0.0);
        } 
        else {
            rightMotor.set(speed);
        }

        //Reach up only if not at the max height of left climber
        if(getLeftHeight() >= MechConstants.MAX_CLIMB_LEFT){
            leftMotor.set(0.0);
        } 
        else {
            leftMotor.set(-speed);
        }
        
    }

    public void climberRetract(double speed){
        
        //retract right climber only if not at bottom/base
        if(rightEncoder.getPosition() <= MechConstants.BASE_CLIMB_RIGHT){
            rightMotor.set(0.0);
        } 
        else {
            rightMotor.set(speed);
        }

        //retract left climber only if not at bottom/base
        if(leftEncoder.getPosition() <= MechConstants.BASE_CLIMB_LEFT){
            leftMotor.set(0.0);
        }
        else {
            leftMotor.set(-speed);
        }
    }

    public void move(double speed){

        speed /= 2;
        if(speed > 0.1){
            climberRetract(speed);
        }
        else if(speed < -0.1){
            climberReach(speed);
        }
        else{
            stop();
        }
    }

    public double getRightHeight(){
        return this.rightEncoder.getPosition();
    }

    public double getLeftHeight(){
        return -this.leftEncoder.getPosition();
    }
    public void stop(){
        rightMotor.set(0);
        leftMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Climber Encoder Value", getLeftHeight());
        SmartDashboard.putNumber("Right Climber Encoder Value", getRightHeight());
    }   


}
