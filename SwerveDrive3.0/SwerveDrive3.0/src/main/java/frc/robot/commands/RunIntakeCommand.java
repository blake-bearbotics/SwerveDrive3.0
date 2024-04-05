package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntakeCommand extends Command{
    private final Intake m_intake;

    public RunIntakeCommand(Intake m_intake) {
        this.m_intake = m_intake;
        this.addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        m_intake.runIntake();
    }

    @Override
    public void execute(){
        
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    public void end(boolean interrupted) {
        m_intake.stopIntake();
    }
}
