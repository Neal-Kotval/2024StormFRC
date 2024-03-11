package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class AutoShoot extends Command {

    private Manipulator manipulator;
    private double velocity;

    public AutoShoot(Manipulator manipulator, double velocity) {
        this.manipulator = manipulator;
        this.velocity = velocity;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        manipulator.velocityShooting(velocity);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Math.abs(Constants.autoShootingVelocity-manipulator.getShootingVelocity()) <= 0.2);
    }

}
