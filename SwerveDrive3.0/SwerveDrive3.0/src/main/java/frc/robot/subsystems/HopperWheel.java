// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class HopperWheel extends SubsystemBase {
  /** Creates a new HopperWheel. */
  private final CANSparkMax indexerMotor;

  public HopperWheel() {
    indexerMotor = new CANSparkMax(OperatorConstants.indexerMotorChannel, MotorType.kBrushless);
  }

  public void runIndexer() {
    indexerMotor.set(-OperatorConstants.intakeSpeed);
  }
  
  public void stopIndexer() {
    indexerMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
