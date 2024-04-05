package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

//how would seth like this to work? Should it be that we can control the climber's position or that it should just climb by itself?
//finish

public class LowerClimberCommand extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Climber climber;
    
    public LowerClimberCommand(Climber climber) {
        this.climber = climber;
        this.addRequirements(climber);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        climber.lower();
    }

    @Override
    public void end(boolean Interupted) {
        climber.keepClimberFromFaling();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
