package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class TimedOutake extends Command {

    private Intake intake;
    private double initialTime;

    public TimedOutake(Intake intake ) {
        this.intake = intake;
        
        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        initialTime = Timer.getFPGATimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.intake(-1);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.intake(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp()-initialTime) >= 0.1;

    }

}
