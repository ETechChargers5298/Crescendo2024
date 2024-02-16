package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.Constants.MechConstants;

public class Climber extends SubsystemBase{

    private CANSparkMax rightMotor;
    private CANSparkMax leftMotor;
    private AbsoluteEncoder leftEncoder;
    private AbsoluteEncoder rightEncoder;
    private static Climber instance;

    private Climber(){
        this.rightMotor = new CANSparkMax(Ports.CLIMB_REACH_MOTOR_PORT, MotorType.kBrushless);
        this.leftMotor = new CANSparkMax(Ports.CLIMB_RETRACT_MOTOR_PORT, MotorType.kBrushless);
        leftEncoder = leftMotor.getAbsoluteEncoder(Type.kDutyCycle);
        rightEncoder = rightMotor.getAbsoluteEncoder(Type.kDutyCycle);

    }

    public static Climber getInstance(){
        if(instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    public void climberReach(double speed){

        if(rightEncoder.getPosition() >= MechConstants.MAX_CLIMB_RIGHT){
            rightMotor.set(0.0);
        } 
        else {
            rightMotor.set(speed);
        }

        if(rightEncoder.getPosition() <= MechConstants.BASE_CLIMB_RIGHT){
            rightMotor.set(0.0);
        } 
        else {
            rightMotor.set(speed);
        }
        
        if(leftEncoder.getPosition() <= MechConstants.BASE_CLIMB_LEFT){
            leftMotor.set(0.0);
        }
        else {
            leftMotor.set(-speed);
        }

        if(leftEncoder.getPosition() >= MechConstants.MAX_CLIMB_LEFT){
            leftMotor.set(0.0);
        } 
        else {
            leftMotor.set(-speed);
        }
        
    }
    
    public double getEncoderPosition(AbsoluteEncoder encoder){
        return encoder.getPosition();
    }

    @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Encoder Value", leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Encoder Value", rightEncoder.getPosition());
  }   


}
