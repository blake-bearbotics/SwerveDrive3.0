package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class PIDTesting extends SubsystemBase{
    CANSparkMax m_turningMotor = new CANSparkMax(OperatorConstants.frontLeftTurningMotorChannel, MotorType.kBrushless);
    ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
            OperatorConstants.frontLeftEncoderPID[0],
            OperatorConstants.frontLeftEncoderPID[1],
            OperatorConstants.frontLeftEncoderPID[2],
          new TrapezoidProfile.Constraints(
            Math.PI, 2 * Math.PI));
    CoreCANcoder m_turningEncoder = new CoreCANcoder(OperatorConstants.frontLeftEncoderChannel);

    public PIDTesting() {
        
    }
    
    public void setWheelPosition() {
        double turnOutput = m_turningPIDController.calculate((m_turningEncoder.getPosition().getValue()) % 1 * 2 * Math.PI, 0);
        m_turningMotor.setVoltage(turnOutput);
    }
}
