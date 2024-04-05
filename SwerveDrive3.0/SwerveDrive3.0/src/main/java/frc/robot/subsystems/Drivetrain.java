package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {

  private Field2d m_field = new Field2d();

  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = 6 * Math.PI; // 1/2 rotation per second
  //check out chassis speeds

  private final Translation2d m_frontLeftLocation = new Translation2d(-OperatorConstants.kWheelBase / 2.0, OperatorConstants.kTrackWidth / 2.0);
  private final Translation2d m_frontRightLocation = new Translation2d(OperatorConstants.kWheelBase / 2.0, OperatorConstants.kTrackWidth / 2.0);
  private final Translation2d m_backLeftLocation = new Translation2d(-OperatorConstants.kWheelBase / 2.0, -OperatorConstants.kTrackWidth / 2.0);
  private final Translation2d m_backRightLocation = new Translation2d(OperatorConstants.kWheelBase / 2.0, -OperatorConstants.kTrackWidth / 2.0);
  //check locations of the wheels


  private final SwerveModule m_frontLeft = new SwerveModule(
    OperatorConstants.frontLeftDriveMotorChannel, 
    OperatorConstants.frontLeftTurningMotorChannel, 
    OperatorConstants.frontLeftEncoderChannel,
    OperatorConstants.frontLeftTurningMotorFeedforward);
  private final SwerveModule m_frontRight = new SwerveModule(
    OperatorConstants.frontRightDriveMotorChannel, 
    OperatorConstants.frontRightTurningMotorChannel, 
    OperatorConstants.frontRightEncoderChannel,
    OperatorConstants.frontRightTurningMotorFeedforward);
  private final SwerveModule m_backLeft = new SwerveModule(
    OperatorConstants.backLeftDriveMotorChannel, 
    OperatorConstants.backLeftTurningMotorChannel, 
    OperatorConstants.backLeftEncoderChannel,
    OperatorConstants.backLeftTurningMotorFeedforward);
  private final SwerveModule m_backRight = new SwerveModule(
    OperatorConstants.backRightDriveMotorChannel, 
    OperatorConstants.backRightTurningMotorChannel, 
    OperatorConstants.backRightEncoderChannel,
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
    m_gyro.reset();  
  
    AutoBuilder.configureHolonomic(
            this::getPose2d, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  
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
    //parameters dictate theoretical robot movement
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));

    SmartDashboard.putData("Field", m_field);
    //the block of code above separates robot movement into module movement
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    //normalizes all robots speeds and sets max speed to 4 m/s
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]); //check this one more time
  }


  public Pose2d getPose2d() {
    return m_odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose2d) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), 
           new SwerveModulePosition[] {
          m_frontLeft.getPosition(), //getPosition() comes from SwerveModule
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()}, 
          pose2d);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return m_kinematics.toChassisSpeeds(m_frontLeft.getState(), 
      m_frontRight.getState(), 
      m_backLeft.getState(), 
      m_backRight.getState());
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    drive(chassisSpeeds.vxMetersPerSecond, 
      chassisSpeeds.vyMetersPerSecond, 
      chassisSpeeds.omegaRadiansPerSecond, 
      false, 
      0.02); //is period correct??
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