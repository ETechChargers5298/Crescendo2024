package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.Constants.MechConstants;

public class Arm extends SubsystemBase{

        private CANSparkMax leftMotor;
        private CANSparkMax rightMotor;
        private AbsoluteEncoder encoder;
        private static Arm instance;

        private Arm(){
            this.leftMotor = new CANSparkMax(Ports.ARM_LEFT, MotorType.kBrushless);
            this.rightMotor = new CANSparkMax(Ports.ARM_RIGHT, MotorType.kBrushless);
            encoder = leftMotor.getAbsoluteEncoder(Type.kDutyCycle);
            encoder.setPositionConversionFactor(360);
            encoder.setZeroOffset(MechConstants.ARM_OFFSET);
            encoder.setInverted(true);
            leftMotor.burnFlash();

          
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
            leftMotor.burnFlash();
          }
          
          public void zeroEncoder() {
            resetValue(encoder.getZeroOffset() + getPosition());

          }
          

          public void pivot(double speed) {
            leftMotor.set(speed);
            rightMotor.set(-speed);
          }
        
          public void stop() {
            leftMotor.set(0);
            rightMotor.set(0);
          }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("PivotAngle",getPosition());
  }   
}
