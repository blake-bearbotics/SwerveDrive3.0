package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Arm;

public class SetAmpArmPosCommand extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final Arm arm;
    
    public SetAmpArmPosCommand(Arm arm) {
        this.arm = arm;
        this.addRequirements(arm);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        arm.setArm(OperatorConstants.ampAngle);
        SmartDashboard.putString("Arm Position", "Amp");
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
