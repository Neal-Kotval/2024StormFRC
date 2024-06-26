// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Telescope;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;


public class TelescopeLeft extends Command {
  private Telescope arm;
  private double power;

  public TelescopeLeft(Telescope arm, double power) {
    this.arm = arm;
    this.power = power;

    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.pull_l(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.pull_l(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}