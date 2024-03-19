package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;

// import javax.swing.text.Position;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

// import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class Manipulator extends SubsystemBase {

    //arm l + r
    TalonFX leftArm = new TalonFX(Constants.LeftArm);
    TalonFX rightArm = new TalonFX(Constants.RightArm);

    //shooter a + b
    TalonFX leftShooter = new TalonFX(Constants.LeftShooter);
    TalonFX rightShooter = new TalonFX(Constants.RightShooter);

    //intake
    TalonSRX intakeMotor = new TalonSRX(Constants.Intake);

    private final PositionVoltage positionRequest;
    private final VelocityVoltage velocityRequest;

    private final TalonFXConfiguration openLoopConfig = new TalonFXConfiguration();

    // DutyCycleEncoder armEncoder = new DutyCycleEncoder(Constants.armEncoder);

    // private final double absolutePosition;

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
        velocityRequest = new VelocityVoltage(0).withSlot(0);

        rightShooter.setInverted(true);
        leftArm.setInverted(true);

        leftArm.setNeutralMode(NeutralModeValue.Brake);
        rightArm.setNeutralMode(NeutralModeValue.Brake);
        intakeMotor.setNeutralMode(NeutralMode.Brake);

        // absolutePosition = armEncoder.getAbsolutePosition();

        //keep in mind TalonFX rotations may be different than Through Bore rotations, shouldnt be problem but verify
        // leftArm.setPosition(absolutePosition);
        // rightArm.setPosition(absolutePosition);
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

    public void intake(double power) {
        intakeMotor.set(ControlMode.PercentOutput,power);
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

    public void setArmEncoder(double pos) {
        leftArm.setPosition(pos);
        rightArm.setPosition(pos);
    }
}
