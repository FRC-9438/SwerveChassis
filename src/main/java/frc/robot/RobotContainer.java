// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Climber.ClimbDownCommand;
import frc.robot.commands.Climber.ClimbUpCommand;
import frc.robot.commands.Intake.IntakeDownCommand;
import frc.robot.commands.Intake.IntakeInCommand;
import frc.robot.commands.Intake.IntakeOutCommand;
import frc.robot.commands.Intake.IntakeUpCommand;
//import frc.robot.commands.RollerClaw.RollerClawInCommand;
//import frc.robot.commands.RollerClaw.RollerClawOutCommand;
import frc.robot.commands.Shooter.ShooterOutCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.climb.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsytem;
//import frc.robot.subsystems.rollerclaw.RollerClawSubsytem;
import frc.robot.subsystems.shooter.ShooterSubsytem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import com.pathplanner.lib.auto.NamedCommands;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */

 
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));


  private final ShooterSubsytem shooter = new ShooterSubsytem();
  //private final RollerClawSubsytem rollerclaw = new RollerClawSubsytem();
  private final IntakeSubsytem intake = new IntakeSubsytem();
  private final ClimberSubsystem climber = new ClimberSubsystem();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorXbox = new CommandXboxController(1);
  final Joystick m_driverXbox = new Joystick(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverXbox.getHID()::getYButtonPressed,
                                                                   driverXbox.getHID()::getAButtonPressed,
                                                                   driverXbox.getHID()::getXButtonPressed,
                                                                   driverXbox.getHID()::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-m_driverXbox.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverXbox.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> -m_driverXbox.getRawAxis(2),
        () -> -m_driverXbox.getRawAxis(3));

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-m_driverXbox.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverXbox.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> -m_driverXbox.getRawAxis(2) * 0.5);

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(-m_driverXbox.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverXbox.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> -m_driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);

    NamedCommands.registerCommand("Intake In", new IntakeInCommand(intake));
    NamedCommands.registerCommand("Intake Out", new IntakeOutCommand(intake));
    NamedCommands.registerCommand("Intake Up", new IntakeUpCommand(intake));
    NamedCommands.registerCommand("Intake Down", new IntakeDownCommand(intake));
    NamedCommands.registerCommand("Shoot", new ShooterOutCommand(shooter, intake));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    driverXbox.b().whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              ));
    

    //shooter                           
    operatorXbox.a().whileTrue(new ShooterOutCommand(shooter, intake));
    //rollerclaw
    //operatorXbox.x().whileTrue(new RollerClawInCommand(rollerclaw));      
    //operatorXbox.y().whileTrue(new RollerClawOutCommand(rollerclaw)); 
    //intake
    operatorXbox.leftBumper().whileTrue(new IntakeUpCommand(intake));
    operatorXbox.rightBumper().whileTrue(new IntakeDownCommand(intake));
    operatorXbox.povRight().whileTrue(new IntakeInCommand(intake));
    operatorXbox.povLeft ().whileTrue(new IntakeOutCommand(intake));
    //climber
    operatorXbox.povUp().whileTrue(new ClimbUpCommand(climber));
    operatorXbox.povDown().whileTrue(new ClimbDownCommand(climber));
                  
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(String a)
  {
    // An example command will be run in autonomous
    Command autoCommand = null;

    switch(a) {
      case "1A1":
        autoCommand = drivebase.getAutonomousCommand("1-A-1 Auto");
        break;
    }

    return autoCommand;
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
