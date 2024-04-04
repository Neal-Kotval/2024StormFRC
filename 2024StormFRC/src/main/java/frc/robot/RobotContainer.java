// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.Delay;
import frc.robot.commands.Intake.PowerIntake;
import frc.robot.commands.Intake.TimedIntake;
import frc.robot.commands.Intake.TimedOutake;
import frc.robot.commands.Manipulator.*;
import frc.robot.commands.Shooter.PowerShoot;
import frc.robot.commands.Shooter.TimedShoot;
import frc.robot.commands.Telescope.*;
import frc.robot.commands.Alignments;
import frc.robot.subsystems.*;

public class RobotContainer {
 
  private double MaxSpeed = 4;
  ; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandXboxController joystick2 = new CommandXboxController(1);



  /* Subsystems */
  private final Swerve drivetrain = TunerConstants.DriveTrain; // My drivetrain
  final Manipulator manipulator = new Manipulator();
  private final Telescope telescope = new Telescope();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final TargetCalcs m_Calcs = new TargetCalcs();


  private final SendableChooser<Command> autoChooser;
  
  //Driver
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop


  private final SwerveRequest.FieldCentricFacingAngle driveFaceinangle = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

  private final PhoenixPIDController turnPID = new PhoenixPIDController(0.3, 0,4 ); //3.2 (10, 1, 0.0);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  public Trigger driverY = new Trigger(joystick.leftBumper());


   


  //Operator
  public Trigger operatorY = new Trigger(joystick2.y());
  public Trigger operatorX = new Trigger(joystick2.x());
  public Trigger operatorA = new Trigger(joystick2.a());
  public Trigger operatorB = new Trigger(joystick2.b());
  public Trigger padUp = new Trigger(joystick2.povUp());
  public Trigger padDown = new Trigger(joystick2.povDown());
  public Trigger padLeft = new Trigger(joystick2.povLeft());
  public Trigger padRight = new Trigger(joystick2.povRight());
  public Trigger leftYAxisActiveUp = new Trigger(()->(joystick2.getLeftY()>0.1));
  public Trigger leftYAxisActiveDown = new Trigger(()->(joystick2.getLeftY()<-0.1));
  public Trigger rightYAxisActiveUp = new Trigger(()->(joystick2.getRightY()>0.1));
  public Trigger rightYAxisActiveDown = new Trigger(()->(joystick2.getRightY()<-0.1));
  public Trigger leftBumper = new Trigger(joystick2.leftBumper());
  public Trigger rightBumper = new Trigger(joystick2.rightBumper());
  public Trigger rightTrigger = new Trigger(()->(joystick2.getRightTriggerAxis()>0.1));
  public Trigger leftTrigger = new Trigger(()->(joystick2.getLeftTriggerAxis()>0.1));
 

  
  


  private void configureBindings() {
    
    /*Swerve Bindings */
    drivetrain.setDefaultCommand
    (
      drivetrain.applyRequest(() -> drive.withVelocityX(joystick.getLeftY() * MaxSpeed)
      .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
      .withRotationalRate(joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
      ).ignoringDisable(true));

    driveFaceinangle.HeadingController = turnPID;
    driveFaceinangle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
  
    // joystick.x().toggleOnTrue(  drivetrain.applyRequest(() -> driveFaceinangle.withVelocityX(0)
    // .withVelocityY(0).withTargetDirection(m_Calcs.AbsRotationToTag(drivetrain.getrobotpose()).minus(drivetrain.Getoffsetroation()))));

    joystick.rightBumper().onTrue(
           Commands.deferredProxy(() -> drivetrain.speakerAlign()));


    

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    
   joystick.povUp().onTrue(new InstantCommand(()->Alignments.sourceAlign()));
   joystick.povLeft().onTrue(new InstantCommand(()->Alignments.ampAlign()));
   joystick.povRight().onTrue(new InstantCommand(()->Alignments.speakerAlign()));
    //operator
    padUp.whileTrue(new MoveArm(manipulator, Constants.armPower));
    padDown.whileTrue(new MoveArm(manipulator, -Constants.armPower));
    // padLeft.whileTrue(new SetSpeed(manipulator,2));
    // padRight.whileTrue(new SetSpeed(manipulator,-2));

    leftYAxisActiveUp.whileTrue(new TelescopeLeft(telescope, 0.2));
    leftYAxisActiveDown.whileTrue(new TelescopeLeft(telescope, -0.2));
    rightYAxisActiveUp.whileTrue(new TelescopeRight(telescope, 0.2));
    rightYAxisActiveDown.whileTrue(new TelescopeRight(telescope, -0.2));
    leftBumper.whileTrue(new PowerIntake(intake, -0.7));
    rightBumper.whileTrue(new PowerIntake(intake, 0.7));
    rightTrigger.whileTrue(new PowerShoot(shooter, -1));
    leftTrigger.whileTrue(new PowerShoot(shooter, 0.1));
    operatorX.onTrue(new SetArm(manipulator, 0));
    operatorY.onTrue(new ArmPos(manipulator, 24));
    // operatorA.onTrue(new ParallelCommandGroup(new SequentialCommandGroup(new Delay(1), new TimedIntake(intake,1)), new TimedShoot(shooter,2)));
    operatorB.onTrue(new ArmPos(manipulator,37 ));

  }



  public RobotContainer() {
    NamedCommands.registerCommand("timedIntake", new TimedIntake(intake,4));
    NamedCommands.registerCommand("timeD", new TimedShoot(shooter,2));
    NamedCommands.registerCommand("setArm90", new SetArm(manipulator, 90));
    NamedCommands.registerCommand("parallelShoot", new ParallelCommandGroup(new SequentialCommandGroup(new Delay(2), new TimedIntake(intake,1)), new TimedShoot(shooter,4)));
    NamedCommands.registerCommand("moveArmFloor", new ArmPos(manipulator, 0).withTimeout(1));
    NamedCommands.registerCommand("moveArmShoot", new ArmPos(manipulator,16).withTimeout(1));
    NamedCommands.registerCommand("timedOutake", new TimedOutake(intake));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
  }
 
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}