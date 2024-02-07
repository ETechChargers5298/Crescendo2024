package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{

        private CANSparkMax motor;
        private AbsoluteEncoder encoder;
        private static Arm instance;

        private Arm(){
            this.motor = new CANSparkMax(0, MotorType.kBrushless);
            encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
        }

        public static Arm getInstance() {
            if (instance == null) {
              instance = new Arm();
            }
            return instance;
          }

          public double getPosition(){
            return encoder.getPosition();
          }

          public void resetValue(double offsetVal) {
            encoder.setZeroOffset(offsetVal);
          }
            
          

          public void pivot(double speed) {
            motor.set(speed);
          }
        
          public void stop() {
            motor.set(0);
          }


    
}
