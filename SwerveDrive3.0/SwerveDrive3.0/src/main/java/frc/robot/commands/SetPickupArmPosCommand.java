package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Arm;

public class SetPickupArmPosCommand extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final Arm arm;
    
    public SetPickupArmPosCommand(Arm arm) {
        this.arm = arm;
        this.addRequirements(arm);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        arm.setArm(OperatorConstants.pickupAngle);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
