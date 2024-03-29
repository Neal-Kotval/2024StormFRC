package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
import frc.robot.subsystems.*;

public class PowerIntake extends Command {

    private Intake intake;
    private double power;

    public PowerIntake(Intake intake, double power) {
        this.intake = intake;
        this.power = power;

        addRequirements(intake);
    }
//
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.intake(power);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.intake(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
