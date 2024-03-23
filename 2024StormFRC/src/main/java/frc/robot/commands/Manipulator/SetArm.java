package frc.robot.commands.Manipulator;

import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
import frc.robot.subsystems.*;

public class SetArm extends Command {

    private Manipulator manipulator;
    private double pos;

    public SetArm(Manipulator manipulator, double pos) {
        this.manipulator = manipulator;
        this.pos = pos;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        manipulator.setArmEncoder(pos);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Math.abs(pos-manipulator.get_arm_enc()) <= 0.01);
    }

}
