package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.OperatorConstants;


public class Arm {
    private final CANSparkMax m_leftArmMotor = new CANSparkMax(OperatorConstants.leftArmMotorChannel, MotorType.kBrushless);
    private final CANSparkMax m_rightArmMotor = new CANSparkMax(OperatorConstants.rightArmMotorChannel, MotorType.kBrushless);

    private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(1);
    private final SimpleMotorFeedforward m_armFeedforward = new SimpleMotorFeedforward(0, 0);

    private final double kArmMaxAngularVelocity = 0.0;
    private final double kArmMaxAngularAcceleration = 0.0;


    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
        OperatorConstants.ArmPID[0],
        OperatorConstants.ArmPID[1],
        OperatorConstants.ArmPID[2],
          new TrapezoidProfile.Constraints(
              kArmMaxAngularVelocity, kArmMaxAngularAcceleration));

    public Arm() {
        
    }

    public void setArm(double armAngle) {
        double armOutput = m_turningPIDController.calculate(armEncoder.getAbsolutePosition(), armAngle);
        double feedForward = m_armFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
        m_leftArmMotor.setVoltage(armOutput + feedForward);
        m_rightArmMotor.setVoltage(armOutput + feedForward);
    }
    // I don't think we need this but I'm leaving it for now just incase
    /*    
    public void stopIntake() {
        m_intakeMotor.set(0.0);
    }
     */
}
