package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VelocityVoltage;

// import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class Shooter extends SubsystemBase {

    //shooter a + b
    TalonFX leftShooter = new TalonFX(Constants.LeftShooter);
    TalonFX rightShooter = new TalonFX(Constants.RightShooter);
    private final VelocityVoltage velocityRequest;
    
    

    public Shooter() {
        
        velocityRequest = new VelocityVoltage(0).withSlot(0);
        rightShooter.setNeutralMode(NeutralModeValue.Brake);
        leftShooter.setNeutralMode(NeutralModeValue.Brake);
        

    }

    public void shoot(double power) {
        leftShooter.set(power);
        rightShooter.set(power);
    }

    public void velocityShooting(double velocity) {
        leftShooter.setControl(velocityRequest.withVelocity(velocity));
        rightShooter.setControl(velocityRequest.withVelocity(velocity));
    }

    public double getShootingVelocity() {
        return leftShooter.getVelocity().getValue();
    }

}
