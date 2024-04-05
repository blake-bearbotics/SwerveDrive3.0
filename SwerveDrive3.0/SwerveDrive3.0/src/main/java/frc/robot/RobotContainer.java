// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AmpScoreCommand;
import frc.robot.commands.HopperWheelCommand;
import frc.robot.commands.LowerClimberCommand;
import frc.robot.commands.PrintArmPositionCommand;
import frc.robot.commands.RaiseClimberCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.SetAmpArmPosCommand;
import frc.robot.commands.SetArmPositionCommand;
import frc.robot.commands.SetPickupArmPosCommand;
import frc.robot.commands.SetSpeakerArmPosCommand;
import frc.robot.commands.SpeakerCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Arm m_arm = new Arm();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Climber m_climber = new Climber();
  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    // Register Name Commands

    NamedCommands.registerCommand("SetSpeakerArmPosCommand", new SetSpeakerArmPosCommand(m_arm));
    NamedCommands.registerCommand("SpeakerCommand", new SpeakerCommand(m_shooter));
    NamedCommands.registerCommand("SetPickupArmPosCommand", new SetPickupArmPosCommand(m_arm));
    NamedCommands.registerCommand("RunIntakeCommand", new RunIntakeCommand(m_intake));
    NamedCommands.registerCommand("SetAmpArmPosCommand", new SetAmpArmPosCommand(m_arm));
    NamedCommands.registerCommand("RaiseClimberCommand", new SetAmpArmPosCommand(m_arm));
    NamedCommands.registerCommand("MoveClimberCommand", new LowerClimberCommand(m_climber));
    NamedCommands.registerCommand("HopperWheelCommand", new HopperWheelCommand(m_shooter));
    NamedCommands.registerCommand("AmpScoreCommand", new AmpScoreCommand(m_shooter));

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
    //Intake
    m_driverController.leftBumper().onTrue(new SetPickupArmPosCommand(m_arm));
    m_driverController.leftTrigger().whileTrue(new RunIntakeCommand(m_intake));

    //Amp
    m_driverController.rightBumper().onTrue(new SetAmpArmPosCommand(m_arm));
    m_driverController.rightTrigger().whileTrue(new AmpScoreCommand(m_shooter));

    //Speaker
    m_driverController.y().onTrue(new SetSpeakerArmPosCommand(m_arm));
    m_driverController.a().whileTrue(new SpeakerCommand(m_shooter));
    m_driverController.b().onTrue(new HopperWheelCommand(m_shooter));

    //testing stuff
    m_driverController.start().whileTrue(new RaiseClimberCommand(m_climber));
    m_driverController.x().whileTrue(new LowerClimberCommand(m_climber));

    //Climber
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
