package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Arm;

public class SetAmpArmPosCommand extends Command{

    public Arm arm;
    public void setAmpArmPos(Arm arm) {
        this.arm = arm;
        this.addRequirements(arm);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        arm.setArm(OperatorConstants.ampAngle);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}