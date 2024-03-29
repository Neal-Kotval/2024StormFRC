package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    CANSparkMax intakeMotor = new CANSparkMax(Constants.Intake, MotorType.kBrushless);

    public Intake() {
        
    }

    public void intake(double power) {
        intakeMotor.set(power);
    }
}
