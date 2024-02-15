package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.Constants.MechConstants;

public class Climber extends SubsystemBase{

    private CANSparkMax right_motor;
    private CANSparkMax left_motor;
    private AbsoluteEncoder l_encoder;
    private AbsoluteEncoder r_encoder;

    private Climber(){
        this.right_motor = new CANSparkMax(Ports.CLIMB_REACH_MOTOR_PORT, MotorType.kBrushless);
        this.left_motor = new CANSparkMax(Ports.CLIMB_RETRACT_MOTOR_PORT, MotorType.kBrushless);
        l_encoder = left_motor.getAbsoluteEncoder(Type.kDutyCycle);
        r_encoder = right_motor.getAbsoluteEncoder(Type.kDutyCycle);

    }

    public void climberReach(double speed){

        if(r_encoder.getPosition() >= MechConstants.MAX_CLIMB_RIGHT){
            right_motor.set(0.0);
        } 
        else {
            right_motor.set(speed);
        }
        
        if(l_encoder.getPosition() >= MechConstants.BASE_CLIMB_RIGHT){
            left_motor.set(0.0);
        }
        else {
            left_motor.set(speed);
        }
        
    }
    
}
