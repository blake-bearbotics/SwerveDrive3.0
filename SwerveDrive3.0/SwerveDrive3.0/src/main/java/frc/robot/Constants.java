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

    //Turning PID values
    public static final double[] turningEncoderPID = {2.0, 0.0, 0.0}; //maybe tune?
    //kp is volts/degrees (in radians)
    //kd is volts*time/degrees

    //Arm Subsystem Constants
    public static final double[] ArmPID = {0.0, 0.0, 0.0}; //continue to adjust
    public static final double[] armFeedforwardConstants = {0.0, 0.361, 1.5}; //continue to adjust

    public static final int leftArmMotorChannel = 6;
    public static final int rightArmMotorChannel = 16;
    public static final int armEncoderChannel = 0;

    public static final double speakerAngle = 0.0;
    public static final double ampAngle = 0.0;
    public static final double pickupAngle = 0.0349;
    public static final double armEncoderOffset = 5.92;

    public static final double kArmMaxAngularVelocity = Math.PI; //figure out something reasonable
    public static final double kArmMaxAngularAcceleration = 0.0; //figure out something reasonable


    //Intake Constants
    public static final int leftIntakeMotorChannel = 17;
    public static final int rightIntakeMotorChannel = 7;
    public static final int indexerMotorChannel = 19;
    public static final double intakeSpeed = 0.0; //figure out this value

    //Shooter Constants
    public static final int leftShooterMotorChannel = 18;
    public static final int rightShooterMotorChannel = 8;
    public static final double ampSpeed = 0.0; //figure out this value
    public static final double speakerSpeed = 0.0; //figure out this value

    //Turning FeedForward Values
    public static final double[] frontLeftTurningMotorFeedforward = {0.125, 0.356};
    public static final double[] frontRightTurningMotorFeedforward = {0.125, 0.356};
    public static final double[] backLeftTurningMotorFeedforward = {0.125, 0.356};
    public static final double[] backRightTurningMotorFeedforward = {0.125, 0.356};

    //Climber Constants
    public static final int climberMotorChannel = 9;
    public static final double climberSpeed = 0.0;

  }
}
