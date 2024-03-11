package frc.robot.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.networktables.GenericEntry;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
//import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.networktables.GenericEntry;
//import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
//import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

//import java.util.Map;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class SwerveModule extends SubsystemBase{
  private static final double kWheelDiameter = 0.1016;
  //private static final int kEncoderResolution = 4096;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder; // RPM
  private final CoreCANcoder m_turningEncoder; //RPS
            
  private static double m_encoderOffset; //need to recalculate these

  private final PIDController m_drivePIDController= new PIDController(0.5,0,0);
  private final ProfiledPIDController m_turningPIDController;
      
  // Gains are for example purposes only - must be determined for your own robot! Figure out what in the world is going on with this bc I don't have the energy right now.
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0, 0);
  private final SimpleMotorFeedforward m_turnFeedforward;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param turningEncoderChannel DIO input for the turning encoder channel A
   * @param ecoderOffset encoder offset
   * @param turningEncoderPID
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      double encoderOffset,
      double[] turningEncoderPID,
      double[] turningFeedforward) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningMotor.setInverted(false);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = new CANcoder(turningEncoderChannel);

    m_encoderOffset = encoderOffset;

    m_turningPIDController = new ProfiledPIDController(
          turningEncoderPID[0],
          turningEncoderPID[1],
          turningEncoderPID[2],
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

    m_turnFeedforward = new SimpleMotorFeedforward(turningFeedforward[0], turningFeedforward[1]);
    // what on earth are the units I'm supposed to use here? Ask Trinh/Burton

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    //in rad
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */

  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity()*kWheelDiameter*Math.PI*60, new Rotation2d((m_turningEncoder.getPosition().getValue())*2*Math.PI)); 
    //SwerveModuleState(speedMetersPerSecond, Rotation2d(radian value)), getVelocity returns RPM
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveEncoder.getPosition() % 1 * kWheelDiameter * Math.PI, new Rotation2d((m_turningEncoder.getPosition().getValue())*2*Math.PI));
    //SwerveModulePosition(distanceMeters, Rotation2d(radian value)), getPosition returns rotations
  }

  public void setPosition(double angle) {
    final double turnOutput =
        m_turningPIDController.calculate((m_turningEncoder.getAbsolutePosition().getValue() - m_encoderOffset) % 1 * 2 * Math.PI, angle);
        m_turningMotor.setVoltage(turnOutput);

  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d((m_turningEncoder.getAbsolutePosition().getValue() - m_encoderOffset) % 1 * 2 * Math.PI);
    //Rotation2d(radian value)CANcoder reads in rot/sec, method Rotation2d requires meters/sec

    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation); 
    // Optimize the reference state to avoid spinning further than 90 degrees
    System.out.println(desiredState);
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getVelocity() * kWheelDiameter * Math.PI * 60, 
                                        state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    // Calculate encoder displacement by printing getPosition while wheel is zeroed (don't trust the Phoenix Tuner)
    final double turnOutput =
        m_turningPIDController.calculate((m_turningEncoder.getAbsolutePosition().getValue() - m_encoderOffset) % 1 * 2 * Math.PI, state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);


    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }
}
