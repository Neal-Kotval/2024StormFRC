package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.States;
import frc.robot.subsystems.Swerve;
import frc.robot.Util.AllianceFlip;

import java.util.ArrayList;
import java.util.List;

public class Alignments {


    // Load the path we want to pathfind to and follow
  //  static PathPlannerPath path = PathPlannerPath.fromPathFile("speaker-align");

    // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
    static  PathConstraints constraints = new PathConstraints(4.5, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    static  PathConstraints slowConstraints = new PathConstraints(0.25, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands



    public static Command ampAlign = AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("amp-align"),
            constraints,
            0
    );

    public static Command sourceAlign(){
        Pose2d pose = new Pose2d(14,1.1, Rotation2d.fromDegrees(180));
        pose = AllianceFlip.apply(pose);
        return AutoBuilder.pathfindToPose(pose,constraints,0);

    }

    public static Command speakerAlign(){
        Pose2d pose = new Pose2d(1.75,FieldConstants.Speaker.centerSpeakerOpening.getY(), Rotation2d.fromDegrees(180));
        pose = AllianceFlip.apply(pose);
        return AutoBuilder.pathfindToPose(pose,constraints,0);
    }}

  