package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class PrintArmPositionCommand extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final Arm arm;
    
    public PrintArmPositionCommand(Arm arm) {
        this.arm = arm;
        this.addRequirements(arm);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        arm.printArmPosition();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
