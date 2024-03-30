package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class TimedShoot extends Command {

    private Shooter shooter;
    private double initialTime;
    private double runTime;

    public TimedShoot(Shooter shooter, double runTime) {
        this.shooter = shooter;
        this.runTime = runTime;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        initialTime = Timer.getFPGATimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.shoot(-1);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.shoot(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp()-initialTime) >= runTime;

    }

}
