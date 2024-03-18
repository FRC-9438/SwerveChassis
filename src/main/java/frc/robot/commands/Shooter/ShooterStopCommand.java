// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsytem;

public class ShooterStopCommand extends Command {
  /** Creates a new ShooterOutCommand. */
   ShooterSubsytem m_Subsytem;

  public ShooterStopCommand(ShooterSubsytem subsytem) {
    // Use addRequirements() here to declare subsystem dependencies.
  m_Subsytem = subsytem;

  addRequirements(subsytem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Subsytem.ShooterStop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
