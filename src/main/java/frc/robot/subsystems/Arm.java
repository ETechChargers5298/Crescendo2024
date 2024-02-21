package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxLimitSwitch.Direction;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.Constants.MechConstants;

public class Arm extends SubsystemBase{

        private CANSparkMax leftMotor;
        private CANSparkMax rightMotor;

        private RelativeEncoder leftEncoder;
        private RelativeEncoder rightEncoder;

        private double posAverage;
        private static Arm instance;

        private Arm(){
          this.leftMotor = new CANSparkMax(Ports.ARM_LEFT, MotorType.kBrushless);
          this.rightMotor = new CANSparkMax(Ports.ARM_RIGHT, MotorType.kBrushless);

          leftMotor.setInverted(true);
          rightMotor.setInverted(false);

          SoftLimitDirection direction = SoftLimitDirection.kReverse;
          leftMotor.enableSoftLimit(direction, true);
          leftMotor.enableSoftLimit(direction, true);

          leftMotor.setSoftLimit(direction, 0.0f);
          rightMotor.setSoftLimit(direction, 0.0f);
          
          leftEncoder = leftMotor.getEncoder();
          rightEncoder = rightMotor.getEncoder();

          leftEncoder.setPositionConversionFactor(360 / (80 * 48 / 22));
          rightEncoder.setPositionConversionFactor(360 / (80 * 48 / 22));

          leftMotor.burnFlash();

        }

        public static Arm getInstance() {
          if (instance == null) {
            instance = new Arm();
          }
          
          return instance;
          }

          public double getPosition(){
            posAverage = (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
            return posAverage;
          }

          public void resetValue() {
            leftEncoder.setPosition(0);
            rightEncoder.setPosition(0);
          }

          public void pivot(double speed) {

            leftMotor.set(speed);
            rightMotor.set(speed);
          }
        
          public void stop() {
            leftMotor.set(0);
            rightMotor.set(0);
          }

  public double getArmAprilAngle() {
    return 16.6 + 9.29 * Camera.getInstance().getX() - 0.645 * Math.pow(Camera.getInstance().getX(), 2);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Value", getPosition());
    SmartDashboard.putNumber("April Arm Angle", getArmAprilAngle());
  }   
}
