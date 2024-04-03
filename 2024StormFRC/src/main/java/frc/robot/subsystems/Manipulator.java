package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

// import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class Manipulator extends SubsystemBase {

    //arm l + r
    TalonFX leftArm = new TalonFX(Constants.LeftArm);
    TalonFX rightArm = new TalonFX(Constants.RightArm);

    
    private DutyCycleOut m_DutyCycle = new DutyCycleOut(0);
   
    private final CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();

    private final double MaxCurrent;

    private final PositionVoltage positionRequest;

    private final VelocityVoltage velocityRequest;

    private final TalonFXConfiguration openLoopConfig = new TalonFXConfiguration();
    

    public Manipulator() {
        
        openLoopConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
        openLoopConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

        MaxCurrent = Constants.Arm.MAX_CURRENT_DRAW;

        leftArm.getConfigurator().apply(openLoopConfig);
        rightArm.getConfigurator().apply(openLoopConfig);

        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.4;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        leftArm.getConfigurator().apply(slot0Configs);
        rightArm.getConfigurator().apply(slot0Configs);

        positionRequest = new PositionVoltage(0).withSlot(0);
        velocityRequest = new VelocityVoltage(0).withSlot(0);
        

        leftArm.setInverted(true);

        leftArm.setNeutralMode(NeutralModeValue.Brake);
        rightArm.setNeutralMode(NeutralModeValue.Brake);
    }

    
    public void ConfigPivotCurrent(){

        TalonFXConfiguration toConfigure = new TalonFXConfiguration();
        
            if (MaxCurrent >= 5){
              //PivotMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,Constants.Arm.MAX_CURRENT_DRAW,Constants.Arm.MAX_CURRENT_DRAW + 5, 0.5));
        
            m_currentLimits.SupplyCurrentLimit = Constants.Arm.MAX_CURRENT_DRAW ; // Limit to 1 amps
            m_currentLimits.SupplyCurrentThreshold = Constants.Arm.MAX_CURRENT_DRAW + 5; // If we exceed 4 amps
            m_currentLimits.SupplyTimeThreshold = .5; // For at least 1 second
            m_currentLimits.SupplyCurrentLimitEnable = true; // And enable it
        
            m_currentLimits.StatorCurrentLimit = 25; // Limit stator to 20 amps
            m_currentLimits.StatorCurrentLimitEnable = true; // And enable it
        
            
        
            toConfigure.CurrentLimits = m_currentLimits;
        
            toConfigure.SoftwareLimitSwitch.withForwardSoftLimitThreshold(245.0);
            toConfigure.SoftwareLimitSwitch.withReverseSoftLimitThreshold(-2);
            toConfigure.SoftwareLimitSwitch.withForwardSoftLimitEnable(false);
            toConfigure.SoftwareLimitSwitch.withReverseSoftLimitEnable(false);
            toConfigure.HardwareLimitSwitch.withForwardLimitEnable(true);
              toConfigure.HardwareLimitSwitch.withReverseLimitEnable(true); 
              toConfigure.HardwareLimitSwitch.withReverseLimitAutosetPositionValue(0);
              toConfigure.HardwareLimitSwitch.withReverseLimitAutosetPositionEnable(true);
        
        
            //toConfigure.MotorOutput.withMotorOutput(NeutralMode.Brake);
            
        
            leftArm.getConfigurator().apply(toConfigure);
            rightArm.getConfigurator().apply(toConfigure);
        
            
        
        
          }
        
              
              else{
               // PivotMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false,40,10,1));
              
        
                m_currentLimits.StatorCurrentLimitEnable = true;
                 m_currentLimits.SupplyCurrentLimitEnable = true;
                 toConfigure.CurrentLimits = m_currentLimits;
        
        
        
        
                 leftArm.getConfigurator().apply(toConfigure);
                rightArm.getConfigurator().apply(toConfigure);
              }
        
              
        
          }
        
          public void setspeed(Double speed){
        leftArm.setControl(m_DutyCycle.withOutput(speed)
        //.withLimitForwardMotion(!m_FwdLimit.get())
        //.withLimitReverseMotion(!m_RevLimit.get())
        );
        rightArm.setControl(m_DutyCycle.withOutput(speed));
        
        
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
        // leftArm.setControl(velocityRequest.withVelocity(power).withFeedForward(0.5));



    }

    public void setArmEncoder(double pos) {
        leftArm.setPosition(pos);
        rightArm.setPosition(pos);
    }
}
