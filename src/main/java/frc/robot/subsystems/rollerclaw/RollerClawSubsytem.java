
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rollerclaw;

import com.revrobotics.CANSparkMax;

import frc.robot.Constants.RollerClawConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class RollerClawSubsytem extends SubsystemBase {
 
  private CANSparkMax rollerMotor;



  /** Creates a new ShooterSubsytem. */
  public RollerClawSubsytem() {
    rollerMotor = new CANSparkMax(RollerClawConstants.ROLLERMOTOR_DEVICE_ID, MotorType.kBrushless);

  }

  public void RollerClawOut () {
    rollerMotor.set(1);
  }
  public void RollerClawIn () {
    rollerMotor.set(-1);
  }
  public void RollerClawStop () {
    rollerMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
