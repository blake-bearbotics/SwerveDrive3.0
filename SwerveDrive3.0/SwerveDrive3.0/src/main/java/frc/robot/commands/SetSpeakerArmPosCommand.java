package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Arm;

public class SetSpeakerArmPosCommand extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final Arm arm;
   
    public SetSpeakerArmPosCommand(Arm arm) {
        this.arm = arm;
        this.addRequirements(arm);
        //ShuffleboardTab armStuffTab = Shuffleboard.getTab("Arm Stuff");
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        arm.setArm(OperatorConstants.speakerAngle);
        SmartDashboard.putString("Arm Position", "Speaker");
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
