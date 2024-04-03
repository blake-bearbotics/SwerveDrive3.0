package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;


public class Climber extends SubsystemBase{
    private final CANSparkMax m_climber = new CANSparkMax(OperatorConstants.leftArmMotorChannel, MotorType.kBrushless);

    private final Encoder climberEncoder = new Encoder(OperatorConstants.climberEncoderChannel[1],OperatorConstants.climberEncoderChannel[1]); //wrong kind of encoder
    private final SimpleMotorFeedforward m_climberFeedforward = new SimpleMotorFeedforward(0, 0);

    //set first
    private final double kArmMaxAngularVelocity = 0.0;
    private final double kArmMaxAngularAcceleration = 0.0;


    private final ProfiledPIDController m_climberPIDController = new ProfiledPIDController(
        OperatorConstants.ArmPID[0],
        OperatorConstants.ArmPID[1],
        OperatorConstants.ArmPID[2],
          new TrapezoidProfile.Constraints(
              kArmMaxAngularVelocity, kArmMaxAngularAcceleration));

    public Climber() {
        
    }

    public void moveClimber(double distance) {
        //figure out the length of the rope that is pulled back and forth (should be half)
        double armOutput = m_climberPIDController.calculate(climberEncoder.getDistance(), distance);
        double feedForward = m_climberFeedforward.calculate(m_climberPIDController.getSetpoint().velocity);
        m_climber.setVoltage(armOutput + feedForward);
    }
        
    public void stopArm() {
        m_climber.set(0.0);
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

