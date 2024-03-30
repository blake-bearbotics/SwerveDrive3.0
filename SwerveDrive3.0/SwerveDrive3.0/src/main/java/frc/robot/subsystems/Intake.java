package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake extends SubsystemBase{
    private final CANSparkMax m_intakeMotor;

    public Intake(int deviceID) {
        m_intakeMotor = new CANSparkMax(deviceID, MotorType.kBrushless); // change devideID to whatever port it is
    }

    public void runIntake() {
        m_intakeMotor.set(0.5);
    }
    // I don't think we need this but I'm leaving it for now just incase
    /*    
    public void stopIntake() {
        m_intakeMotor.set(0.0);
    }
     */
}
