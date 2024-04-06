package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperWheel;

public class HopperWheelCommand extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final HopperWheel hopperWheel;
    
    public HopperWheelCommand(HopperWheel hopperWheel) {
        this.hopperWheel = hopperWheel;
        this.addRequirements(hopperWheel);
    }

    @Override
    public void initialize(){
        hopperWheel.runIndexer();
    }

    @Override
    public void execute(){
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    public void end(boolean interrupted) {
        hopperWheel.stopIndexer();
    }
}
