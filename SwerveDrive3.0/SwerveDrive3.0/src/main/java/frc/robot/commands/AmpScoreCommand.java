package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Shooter;

public class AmpScoreCommand extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final Shooter shooter;
    
    public AmpScoreCommand(Shooter shooter) {
        this.shooter = shooter;
        this.addRequirements(shooter);
    }

    @Override
    public void initialize(){
        shooter.runIndexer();
        shooter.runShooter(OperatorConstants.ampSpeed);
    }

    @Override
    public void execute(){
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    public void end(boolean interrupted) {
        shooter.stopShooter();
    }
}
