package frc.robot.subsystems;

import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TelescopingArm extends SubsystemBase{
    
    TalonFX LeftClimb = new TalonFX(Constants.LeftClimb);
    TalonFX RightClimb = new TalonFX(Constants.RightClimb);
    
    public TelescopingArm() {
        // Configuration
        LeftClimb.setNeutralMode(NeutralModeValue.Brake);
        RightClimb.setNeutralMode(NeutralModeValue.Brake);
    }

    public void pull_l(double d) {
        LeftClimb.set(d);
       }
    public void pull_r(double power) {
        RightClimb.set(power);
    }

    
}
