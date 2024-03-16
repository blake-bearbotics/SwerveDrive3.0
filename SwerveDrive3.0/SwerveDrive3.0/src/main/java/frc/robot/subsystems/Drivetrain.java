package frc.robot.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
//import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.networktables.StructArrayPublisher;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.smartdashboard.Field2d;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

//import edu.wpi.first.wpilibj.AnalogGyro;
import com.ctre.phoenix6.hardware.Pigeon2;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  public static final double kMaxSpeed = 4.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(-OperatorConstants.kWheelBase / 2.0, OperatorConstants.kTrackWidth / 2.0);
  private final Translation2d m_frontRightLocation = new Translation2d(OperatorConstants.kWheelBase / 2.0, OperatorConstants.kTrackWidth / 2.0);
  private final Translation2d m_backLeftLocation = new Translation2d(-OperatorConstants.kWheelBase / 2.0, -OperatorConstants.kTrackWidth / 2.0);
  private final Translation2d m_backRightLocation = new Translation2d(OperatorConstants.kWheelBase / 2.0, -OperatorConstants.kTrackWidth / 2.0);


  private final SwerveModule m_frontLeft = new SwerveModule(
    OperatorConstants.frontLeftDriveMotorChannel, 
    OperatorConstants.frontLeftTurningMotorChannel, 
    OperatorConstants.frontLeftEncoderChannel, 
    OperatorConstants.frontLeftEncoderOffset, 
    OperatorConstants.frontLeftEncoderPID, 
    OperatorConstants.frontLeftTurningMotorFeedforward);
  private final SwerveModule m_frontRight = new SwerveModule(
    OperatorConstants.frontRightDriveMotorChannel, 
    OperatorConstants.frontRightTurningMotorChannel, 
    OperatorConstants.frontRightEncoderChannel, 
    OperatorConstants.frontRightEncoderOffset, 
    OperatorConstants.frontRightEncoderPID, 
    OperatorConstants.frontRightTurningMotorFeedforward);
  private final SwerveModule m_backLeft = new SwerveModule(
    OperatorConstants.backLeftDriveMotorChannel, 
    OperatorConstants.backLeftTurningMotorChannel, 
    OperatorConstants.backLeftEncoderChannel, 
    OperatorConstants.backLeftEncoderOffset, 
    OperatorConstants.backLeftEncoderPID, 
    OperatorConstants.backLeftTurningMotorFeedforward);
  private final SwerveModule m_backRight = new SwerveModule(
    OperatorConstants.backRightDriveMotorChannel, 
    OperatorConstants.backRightTurningMotorChannel, 
    OperatorConstants.backRightEncoderChannel, 
    OperatorConstants.backRightEncoderOffset, 
    OperatorConstants.backRightEncoderPID, 
    OperatorConstants.backRightTurningMotorFeedforward);

  private final Pigeon2 m_gyro = new Pigeon2(5);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

   private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });


  public Drivetrain() {
    m_gyro.reset();  }

  public void setWheelPosition() {
    m_frontLeft.setPosition(0);
    m_frontRight.setPosition(0);
    m_backLeft.setPosition(Math.PI);
    m_backRight.setPosition(0);
  }
  //don't know if this is necessary
  /**
   * Method to drive the robot using joystick info.
   * Purely theoretical, not impacted by what the robot position actually is
   * TO-DO: invert x-axis
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    System.out.println(ySpeed);
    //parameters dictate theoretical robot movement
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
    //System.out.println(swerveModuleStates[1]);
    //the block of code above separates robot movement into module movement
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    //normalizes all robots speeds and sets max speed to 4 m/s
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]); //check this one more time

    //Shuffleboard.getTab("Swerve Module States").add()
  }

  
  

  /** Updates the field relative position of the robot. */
  //what should be the units of the Pigeon?
  //reads where robot is
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(), //units of gyro: do they have to match the units of getPosition? 
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(), //getPosition() comes from SwerveModule
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }
}