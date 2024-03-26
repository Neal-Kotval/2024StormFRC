package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants;

public class Intake {
    CANSparkMax intakeMotor = new CANSparkMax(Constants.Intake, MotorType.kBrushless);

    public Intake() {
        
    }

    public void intake(double power) {
        intakeMotor.set(power);
    }
}
