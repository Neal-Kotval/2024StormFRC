package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


public class Delay extends Command {

    private double delayTime;
    private double initialTime;

    public Delay(double delayTime) {
        this.delayTime = delayTime;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        initialTime = Timer.getFPGATimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp()-initialTime) >= delayTime;

    }

}
