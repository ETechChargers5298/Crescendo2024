package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{

        private CANSparkMax motor;
        private static Arm instance;

        private Arm(){
            this.motor = new CANSparkMax(0, MotorType.kBrushless);
        }

        public static Arm getInstance() {
            if (instance == null) {
              instance = new Arm();
            }
            return instance;
          }

          public void pivot(double speed) {
            motor.set(speed);
          }
        
          public void stop() {
            motor.set(0);
          }


    
}
