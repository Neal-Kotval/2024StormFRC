package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;

// import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class Manipulator extends SubsystemBase {

    //arm l + r
    TalonFX leftArm = new TalonFX(Constants.LeftArm);
    TalonFX rightArm = new TalonFX(Constants.RightArm);


    //intake
    // CANSparkMax intakeMotor = new CANSparkMax(Constants.Intake, MotorType.kBrushless);

    private final PositionVoltage positionRequest;

    private final TalonFXConfiguration openLoopConfig = new TalonFXConfiguration();

    public Manipulator() {
        
        openLoopConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
        openLoopConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

        leftArm.getConfigurator().apply(openLoopConfig);
        rightArm.getConfigurator().apply(openLoopConfig);

        var slot0Configs = new Slot0Configs();

        slot0Configs.kP = 0.4;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        leftArm.getConfigurator().apply(slot0Configs);
        rightArm.getConfigurator().apply(slot0Configs);

        positionRequest = new PositionVoltage(0).withSlot(0);

        leftArm.setInverted(true);

        leftArm.setNeutralMode(NeutralModeValue.Brake);
        rightArm.setNeutralMode(NeutralModeValue.Brake);
    }

    public double get_arm_enc() {
       return  leftArm.getPosition().getValue();
    }

    public void arm_to_pos(double pos) {
        leftArm.setControl(positionRequest.withPosition(pos));
        rightArm.setControl(positionRequest.withPosition(pos));
    }

    public void move_arm(double power) {
        leftArm.set(power);
        rightArm.set(power);
    }

    public void setArmEncoder(double pos) {
        leftArm.setPosition(pos);
        rightArm.setPosition(pos);
    }
}
