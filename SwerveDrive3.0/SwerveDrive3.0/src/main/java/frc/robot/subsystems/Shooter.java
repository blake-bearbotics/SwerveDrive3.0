package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase{
    private final CANSparkMax leftShooterMotor;
    private final CANSparkMax rightShooterMotor;
    private final CANSparkMax indexerMotor;


    public Shooter() {
       leftShooterMotor = new CANSparkMax(OperatorConstants.leftShooterMotorChannel, MotorType.kBrushless);
       rightShooterMotor = new CANSparkMax(OperatorConstants.rightShooterMotorChannel, MotorType.kBrushless);
       indexerMotor = new CANSparkMax(OperatorConstants.indexerMotorChannel, MotorType.kBrushless);
    }

    public void runShooter(double speed) {
        leftShooterMotor.set(speed);
        rightShooterMotor.set(speed);
    }
     
    public void stopShooter() {
        leftShooterMotor.set(0.0);
        rightShooterMotor.set(0.0);
    }

    public void runIndexer() {
        indexerMotor.set(OperatorConstants.intakeSpeed); // make its own subsystem?
    }


    
    /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command armCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
