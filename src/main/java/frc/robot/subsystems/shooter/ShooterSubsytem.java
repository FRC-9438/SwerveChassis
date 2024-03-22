
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsytem extends SubsystemBase {
 
  private CANSparkMax shooterrightMotor;
  private CANSparkMax shooterleftMotor;



  /** Creates a new ShooterSubsytem. */
  public ShooterSubsytem() {
    //feederMotor = new CANSparkMax(ShooterConstants.FEEDERMOTOR_DEVICE_ID, MotorType.kBrushed);
    shooterleftMotor = new CANSparkMax(ShooterConstants.SHOOTERMOTORLEFT_DEVICE_ID, MotorType.kBrushless);
    shooterrightMotor = new CANSparkMax(ShooterConstants.SHOOTERMOTORRIGHT_DEVICE_ID, MotorType.kBrushless);
    shooterleftMotor.setIdleMode(IdleMode.kCoast);
    shooterrightMotor.setIdleMode(IdleMode.kCoast);
  }

  public void ShooterOut () {
    shooterleftMotor.set(1);
    shooterrightMotor.set(1);
  }
    public void ShooterStop () {
    shooterleftMotor.set(0);
    shooterrightMotor.set(0);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
