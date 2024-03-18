
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;

import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsytem extends SubsystemBase {
 
  private CANSparkMax feederMotor;
  private CANSparkMax shooterMotor;



  /** Creates a new ShooterSubsytem. */
  public ShooterSubsytem() {
    feederMotor = new CANSparkMax(ShooterConstants.FEEDERMOTOR_DEVICE_ID, MotorType.kBrushed);
    shooterMotor = new CANSparkMax(ShooterConstants.SHOOTERMOTOR_DEVICE_ID, MotorType.kBrushed);

  }

  public void ShooterOut () {
    shooterMotor.set(1);
  }
    public void ShooterStop () {
    shooterMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
