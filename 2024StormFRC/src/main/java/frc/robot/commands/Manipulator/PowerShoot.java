package frc.robot.commands.Manipulator;

import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
import frc.robot.subsystems.*;

public class PowerShoot extends Command {

    private Manipulator manipulator;
    private double power;

    public PowerShoot(Manipulator manipulator, double power) {
        this.manipulator = manipulator;
        this.power = power;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        manipulator.shoot(-power);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        manipulator.shoot(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
