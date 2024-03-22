// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimberSubsystem;

public class ClimbDownCommand extends Command {
  /** Creates a new ShooterOutCommand. */
   ClimberSubsystem m_Subsytem;

  public ClimbDownCommand(ClimberSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
  m_Subsytem = subsystem;

  addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Subsytem.ClimberIn();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_Subsytem.ClimberStop();
  }
}
