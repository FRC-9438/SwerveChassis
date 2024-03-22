// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsytem;
import frc.robot.subsystems.intake.IntakeSubsytem;
import edu.wpi.first.wpilibj.Timer;

public class ShooterOutCommand extends Command {
  /** Creates a new ShooterOutCommand. */
   ShooterSubsytem m_Subsytem;
   IntakeSubsytem m_Intake;
   Timer timer = new Timer();

  public ShooterOutCommand(ShooterSubsytem subsytem, IntakeSubsytem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
  m_Subsytem = subsytem;
  m_Intake = intake;

  addRequirements(subsytem, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Subsytem.ShooterOut();
    timer.start();
    timer.reset();
  }

  @Override
  public void execute() {
    if (timer.get() < 2) {
      m_Subsytem.ShooterOut();
    } else {
      m_Subsytem.ShooterOut();
      m_Intake.IntakeOut();                                                                                 
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_Subsytem.ShooterStop();
    m_Intake.IntakeStop();
  }
}
