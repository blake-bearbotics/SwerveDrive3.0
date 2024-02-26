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
    
    public static final double kWheelBase = 0.5588;
    public static final double kTrackWidth = 0.5588;
    
    public static final int frontLeftDriveMotorChannel = 4;
    public static final int frontRightDriveMotorChannel = 1;
    public static final int backLeftDriveMotorChannel = 2;
    public static final int backRightDriveMotorChannel = 3;

    public static final int frontLeftTurningMotorChannel = 14;
    public static final int frontRightTurningMotorChannel = 11;
    public static final int backLeftTurningMotorChannel = 12;
    public static final int backRightTurningMotorChannel = 13;

    public static final int frontLeftEncoderChannel = 24;
    public static final int frontRightEncoderChannel = 21;
    public static final int backLeftEncoderChannel = 22;
    public static final int backRightEncoderChannel = 23;

    //must put in rotations
    public static final double frontLeftEncoderOffset = -0.357422;//-6.504; 
    public static final double frontRightEncoderOffset = 0.168213;//342.334;
    public static final double backLeftEncoderOffset = -0.367920;//53.174;
    public static final double backRightEncoderOffset = -0.447266;//188.789;

    public static final double[] frontLeftEncoderPID = {0.0, 0.0, 0.0};
    public static final double[] frontRightEncoderPID = {0.0, 0.0, 0.0};
    public static final double[] backLeftEncoderPID = {0.0, 0.0, 0.0};
    public static final double[] backRightEncoderPID = {0.0, 0.0, 0.0};

    public static final double[] frontLeftTurningMotorFeedforward = {0.3, 19.78};
    public static final double[] frontRightTurningMotorFeedforward = {0.3,20.79};
    public static final double[] backLeftTurningMotorFeedforward = {0.3, 20};
    public static final double[] backRightTurningMotorFeedforward = {0.3, 20.3};

  }
}
