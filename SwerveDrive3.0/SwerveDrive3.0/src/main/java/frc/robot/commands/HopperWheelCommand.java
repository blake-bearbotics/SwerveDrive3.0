package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class HopperWheelCommand extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final Shooter shooter;
    
    public HopperWheelCommand(Shooter shooter) {
        this.shooter = shooter;
        this.addRequirements(shooter);
    }

    @Override
    public void initialize(){
        shooter.runIndexer();
    }

    @Override
    public void end(boolean Interupted) {

    }

    @Override
    public void execute(){
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    public void end​(boolean interrupted) {
        shooter.stopShooter();
    }
}
