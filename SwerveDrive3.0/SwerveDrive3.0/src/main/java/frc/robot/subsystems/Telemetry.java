package frc.robot.subsystems;

//import pabeles.concurrency.ConcurrencyOps.Reset;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class Telemetry {
   
    public Telemetry () {
        Shuffleboard.getTab("Testing2.0")
            .add("PID values", 1)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .getEntry();
    }
    }
