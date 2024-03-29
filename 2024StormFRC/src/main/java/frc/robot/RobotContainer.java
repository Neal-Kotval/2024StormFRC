// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.Manipulator.*;
import frc.robot.commands.Telescope.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  private double MaxSpeed = 2.5; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1. * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandXboxController joystick2 = new CommandXboxController(1);

  /* Subsystems */
  private final Swerve drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Manipulator manipulator = new Manipulator();
  private final Telescope telescope = new Telescope();


  private final SendableChooser<Command> autoChooser;
  
  
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  // Triggers for Controller 2
  public Trigger operatorY = new Trigger(joystick2.y());
  public Trigger operatorX = new Trigger(joystick2.x());
  public Trigger operatorA = new Trigger(joystick2.a());
  public Trigger padUp = new Trigger(joystick2.povUp());
  public Trigger padDown = new Trigger(joystick2.povDown());
  public Trigger leftYAxisActiveUp = new Trigger(()->(joystick2.getLeftY()>0.1));
  public Trigger leftYAxisActiveDown = new Trigger(()->(joystick2.getLeftY()<-0.1));
  public Trigger rightYAxisActiveUp = new Trigger(()->(joystick2.getRightY()>0.1));
  public Trigger rightYAxisActiveDown = new Trigger(()->(joystick2.getRightY()<-0.1));
  public Trigger leftBumper = new Trigger(joystick2.leftBumper());
  public Trigger rightBumper = new Trigger(joystick2.rightBumper());
  public Trigger rightTrigger = new Trigger(()->(joystick2.getRightTriggerAxis()>0.1));



  private void configureBindings() {
    
    /*Swerve Bindings */
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);



    padUp.whileTrue(new MoveArm(manipulator, Constants.armPower));
    padDown.whileTrue(new MoveArm(manipulator, -Constants.armPower));

    leftYAxisActiveUp.whileTrue(new TelescopeLeft(telescope, 0.2));
    leftYAxisActiveDown.whileTrue(new TelescopeLeft(telescope, -0.2));
    rightYAxisActiveUp.whileTrue(new TelescopeRight(telescope, 0.2));
    rightYAxisActiveDown.whileTrue(new TelescopeRight(telescope, -0.2));
    leftBumper.whileTrue(new PowerIntake(manipulator, -0.7));
    rightBumper.whileTrue(new PowerIntake(manipulator, 0.7));
    rightTrigger.whileTrue(new PowerShoot(manipulator, 0.7));
    operatorX.onTrue(new SetArm(manipulator, 0));
    operatorY.onTrue(new ArmPos(manipulator, 16));
    operatorA.onTrue(new TimedIntake(manipulator));


  }



  public RobotContainer() {
    NamedCommands.registerCommand("timedIntake", new ParallelCommandGroup(new TimedIntake(manipulator), new TimedShoot(manipulator)) );
    // NamedCommands.registerCommand("timedShoot", new TimedShoot(manipulator));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();

   
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}