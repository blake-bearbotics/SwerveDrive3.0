package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake {
    private final CANSparkMax m_intakeMotor;

    public Intake() {
        m_intakeMotor = new CANSparkMax(1, MotorType.kBrushless); // change devideID to whatever port it is
    }

    public void runIntake() {
        m_intakeMotor.set(0.5);
    }

    public void stopIntake() {
        m_intakeMotor.set(0.0);
    }
}
