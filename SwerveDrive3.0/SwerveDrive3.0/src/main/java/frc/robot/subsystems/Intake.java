package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake extends SubsystemBase{
    private final CANSparkMax leftIntakeMotor;
    private final CANSparkMax rightIntakeMotor;


    public Intake() {
       leftIntakeMotor = new CANSparkMax(OperatorConstants.leftIntakeMotorChannel, MotorType.kBrushless);
       rightIntakeMotor = new CANSparkMax(OperatorConstants.rightIntakeMotorChannel, MotorType.kBrushless);
    }

    public void runIntake() {
        leftIntakeMotor.set(OperatorConstants.intakeSpeed);
        rightIntakeMotor.set(OperatorConstants.intakeSpeed);
    }
     
    public void stopIntake() {
        leftIntakeMotor.set(0.0);
        rightIntakeMotor.set(0.0);
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
