
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.ClimberConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ClimberSubsystem extends SubsystemBase {
 
  private CANSparkMax climbMotor;



  /** Creates a new ShooterSubsytem. */
  public ClimberSubsystem() {
    climbMotor = new CANSparkMax(ClimberConstants.CLIMBERMOTOR_DEVICE_ID, MotorType.kBrushless);
    climbMotor.setIdleMode(IdleMode.kBrake);
    climbMotor.burnFlash();

  }

  public void ClimberIn () {
    climbMotor.set(-1);
  }
  public void ClimberOut () {
    climbMotor.set(1);
  }
  public void ClimberStop () {
    climbMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
