package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;


public class Arm extends SubsystemBase{
    private final CANSparkMax m_leftArmMotor = new CANSparkMax(OperatorConstants.leftArmMotorChannel, MotorType.kBrushless);
    private final CANSparkMax m_rightArmMotor = new CANSparkMax(OperatorConstants.rightArmMotorChannel, MotorType.kBrushless);

    private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(OperatorConstants.armEncoderChannel);
    private final ArmFeedforward m_armFeedforward = new ArmFeedforward(
        OperatorConstants.armFeedforwardConstants[0], 
        OperatorConstants.armFeedforwardConstants[1], 
        OperatorConstants.armFeedforwardConstants[2]);

    private final ProfiledPIDController m_armPIDController = new ProfiledPIDController(
        OperatorConstants.ArmPID[0],
        OperatorConstants.ArmPID[1],
        OperatorConstants.ArmPID[2],
          new TrapezoidProfile.Constraints(
              OperatorConstants.kArmMaxAngularVelocity, OperatorConstants.kArmMaxAngularAcceleration));

    public Arm() {
        
    }

    public double getArmPosition() {
        double armPosition = 0.0;

        if ((armEncoder.getAbsolutePosition() * 2 * Math.PI) - OperatorConstants.armEncoderOffset < 0) {
            armPosition = 2 * Math.PI + ((armEncoder.getAbsolutePosition() * 2 * Math. PI) - OperatorConstants.armEncoderOffset);
        } else {
            armPosition = (armEncoder.getAbsolutePosition() * 2 * Math. PI) - OperatorConstants.armEncoderOffset;
        }
        return armPosition;
    }

    public void setArm(double armAngle) {
        double armOutput = m_armPIDController.calculate(getArmPosition(), armAngle);
        double feedForward = m_armFeedforward.calculate(m_armPIDController.getSetpoint().position, m_armPIDController.getSetpoint().velocity); //figure out the setpoint velocity here
        m_leftArmMotor.setVoltage(-(armOutput + feedForward));
        m_rightArmMotor.setVoltage((armOutput + feedForward));
        SmartDashboard.putNumber("Arm Angle", getArmPosition());
        SmartDashboard.putNumber("Arm Outupt", armOutput);
        SmartDashboard.putNumber("Arm Feed Forward", feedForward);
    }
        
    public void stopArm() {
        m_leftArmMotor.set(0.0);
        m_rightArmMotor.set(0.0);

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
    // the pid command needs to be a periodic function (right?), so figure out how to make it one (or see if it already is?)
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

