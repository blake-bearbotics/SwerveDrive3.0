package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;


public class Climber extends SubsystemBase{
    //finish coding climber!!
    private final CANSparkMax m_climber = new CANSparkMax(OperatorConstants.climberMotorChannel, MotorType.kBrushless);

    private final SimpleMotorFeedforward m_climberFeedforward = new SimpleMotorFeedforward(0, 0);

    public Climber() {
        
    }

    public void raise() {
        m_climber.set(OperatorConstants.climberSpeed);
    }

    public void lower() {
        m_climber.set(-OperatorConstants.climberSpeed);
    }

    public void keepClimberFromFaling(){
        double feedForward = m_climberFeedforward.calculate(0);
        m_climber.setVoltage(feedForward); //maybe just convert to a voltage that keeps the robot up
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

