// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    
    //Drivebase Constants
    public static final double kWheelBase = 0.5588; //meters
    public static final double kTrackWidth = 0.5588; //meters
    public static final double kWheelRadius = 0.0508; //meters
    public static final double kDriveMotorGearRatio = 6.75; //6.75 motor rot = 1 wheel rot
    public static final double kTurningMotorGearRatio = 150/7; //150/7 motor rot = 1 wheel rot

    //Drive Motor IDs
    public static final int frontLeftDriveMotorChannel = 4;
    public static final int backLeftDriveMotorChannel = 3;
    public static final int frontRightDriveMotorChannel = 1;
    public static final int backRightDriveMotorChannel = 2;

    //Turning Motor IDs
    public static final int frontLeftTurningMotorChannel = frontLeftDriveMotorChannel + 10;
    public static final int backLeftTurningMotorChannel = backLeftDriveMotorChannel + 10;
    public static final int frontRightTurningMotorChannel = frontRightDriveMotorChannel + 10;
    public static final int backRightTurningMotorChannel = backRightDriveMotorChannel + 10;

    //Encoder IDs left and right screwed up?
    public static final int frontLeftEncoderChannel = frontLeftDriveMotorChannel + 20;
    public static final int backLeftEncoderChannel = backLeftDriveMotorChannel + 20;
    public static final int frontRightEncoderChannel = frontRightDriveMotorChannel + 20;
    public static final int backRightEncoderChannel = backRightDriveMotorChannel + 20;

    public static final int leftArmMotorChannel = 0;
    public static final int rightArmMotorChannel = 0;

    //Swerve CANCoder Sensor offsets
    //CHANGE TO 0 first, reset the sensor, 
    //PHYSICALLY zero out the motor 
    //place the OPPOSITE of the value
    public static double frontLeftEncoderOffset = 0;
    public static double frontRightEncoderOffset = 0;
    public static double backLeftEncoderOffset = 0;
    public static double backRightEncoderOffset = 0;

    //Constants for conversion maths
    public static final double secondsPer100ms = .1;
    public static final double turningEncoderTicksPerRotation = 4096;
    public static final double turningEncoderRotationsPerTick= 360/turningEncoderTicksPerRotation;
    public static final double TICKSperTALONFX_Rotation = 2048;
    //public static final double DRIVE_MOTOR_TICKSperREVOLUTION = DRIVE_MOTOR_GEARING*TICKSperTALONFX_Rotation;
    public static final double metersPerWheelRevolution = 2*Math.PI*kWheelRadius;
    public static final double metersPerRobotRevolution =  2*Math.PI*(kWheelBase/2*Math.sqrt(2));
    public static final double maxSpeedMetersPerSecond = 0.0; //figure this out later
    public static final double maxSpeedRadiansPerSecond = maxSpeedMetersPerSecond/metersPerRobotRevolution*(2*Math.PI);
    //public static final double TICKSperTALONFX_DEGREE = TICKSperTALONFX_Rotation*STEERING_MOTOR_GEARING/360;

    //Swerve Drive Constants
    public static final boolean defaultFieldRelativeDrive = true;
    public static final double robotHoldAngleKP = 10; //Start at .7 and see where you go from there
    public static final boolean defaultHoldRobotAngle = true;
	  public static final double defaultHoldRobotAngleSetpoint = 0; 

    //Turning PID values
    public static final double[] frontLeftEncoderPID = {0.2, 0.0, 0.2};
    public static final double[] frontRightEncoderPID = {0.2, 0.0, 0.2};
    public static final double[] backLeftEncoderPID = {0.2, 0.0, 0.2};
    public static final double[] backRightEncoderPID = {0.2, 0.0, 0.2};

    //Systems PID Values
    public static final double[] ArmPID = {0.0, 0.0, 0.0};

    //Turning FeedForward Values
    public static final double[] frontLeftTurningMotorFeedforward = {0.15, 0.01}; //start ~2.5
    public static final double[] frontRightTurningMotorFeedforward = {0.15, 0.01};
    public static final double[] backLeftTurningMotorFeedforward = {0.15, 0.01};
    public static final double[] backRightTurningMotorFeedforward = {0.15, 0.01};

    //CTRE CAN-based constants (shouldn't need to change these)
    public static final int kDefaultPIDSlotID = 0;
    public static final int kDefaultTimeout = 30;//milliseconds
    public static final int kDefaultClosedLoopError = 1; //degrees 

  }
}
