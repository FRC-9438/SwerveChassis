// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RollerClaw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.rollerclaw.RollerClawSubsytem;

public class RollerClawInCommand extends Command {
  /** Creates a new ShooterOutCommand. */
   RollerClawSubsytem m_Subsytem;

  public RollerClawInCommand(RollerClawSubsytem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
  m_Subsytem = subsystem;

  addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Subsytem.RollerClawIn();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_Subsytem.RollerClawStop();
  }
}
