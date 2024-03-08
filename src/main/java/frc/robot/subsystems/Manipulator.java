package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;

// import javax.swing.text.Position;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class Manipulator extends SubsystemBase {

    //checks if note is in intake
   

    //arm l + r
    TalonFX arm_l = new TalonFX(Constants.LeftArm);
    TalonFX arm_r = new TalonFX(Constants.RightArm);

    //shooter a + b
    TalonFX LeftShooter = new TalonFX(Constants.LeftShooter);
    TalonFX RightShooter = new TalonFX(Constants.RightShooter);

    //intake
    TalonSRX intakeMotor = new TalonSRX(Constants.Intake);


    public final  double kARM_FLOOR_POS = -27.846;  // intaking
    public final double kARM_FENDER_POS = 0;  // close shot
    public final double kARM_START_POS = 0;  // start config
    public final double kARM_AMP_POS   = 0;  // amp scoring

    private final PositionVoltage m_position_request;
    private final VelocityVoltage m_velocity_request;


    private final TalonFXConfiguration openLoopConfig = new TalonFXConfiguration();

    public Manipulator() {
        

        openLoopConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
        openLoopConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

        arm_l.getConfigurator().apply(openLoopConfig);
        arm_r.getConfigurator().apply(openLoopConfig);

        var slot0Configs = new Slot0Configs();

        slot0Configs.kP = 0.1;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        arm_l.getConfigurator().apply(slot0Configs);
        arm_r.getConfigurator().apply(slot0Configs);

        m_position_request = new PositionVoltage(0).withSlot(0);
        m_velocity_request = new VelocityVoltage(0).withSlot(0);

        
        RightShooter.setInverted(true);
        arm_l.setInverted(true);

        // arm_r.setControl(new StrictFollower(arm_l.getDeviceID()));

        arm_l.setNeutralMode(NeutralModeValue.Brake);
        arm_r.setNeutralMode(NeutralModeValue.Brake);
        intakeMotor.setNeutralMode(NeutralMode.Brake);

    }

    public double get_arm_enc() {
        //returns in rotations
       return  arm_l.getPosition().getValue();
    }

   
    public void arm_to_pos(double pos) {
            
        arm_l.setControl(m_position_request.withPosition(pos));
        arm_r.setControl(m_position_request.withPosition(pos));
    }

    public void move_arm(double power) {

        arm_l.set(power);
        arm_r.set(power);
      // -ve, as motors are pointing in opposite directions
    }

    public void intake(double power) {
        intakeMotor.set(ControlMode.PercentOutput,power);
    }


    public void shoot(double power) {
        LeftShooter.set(power);
        RightShooter.set(power);
    }

    public void velocityShooting(double velocity) {
        LeftShooter.setControl(m_velocity_request.withVelocity(velocity));
        RightShooter.setControl(m_velocity_request.withVelocity(velocity));
    }



    public void zeroEncoder() {
        arm_l.setPosition(0);
        arm_r.setPosition(0);
    }
}
