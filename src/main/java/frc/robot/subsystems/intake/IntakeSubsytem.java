
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RollerClawConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsytem extends SubsystemBase {
 
  private CANSparkMax intaketopMotor;
  private CANSparkMax intakebottomMotor;
  private CANSparkMax intakerotationMotor;
  
  private SparkPIDController m_PidController;
  private RelativeEncoder m_Encoder;
  public double kP, kI, kD, kIz, kMaxOutput, kFF;
  public double kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;


  /** Creates a new ShooterSubsytem. */
  public IntakeSubsytem() {
    intaketopMotor = new CANSparkMax(IntakeConstants.INTAKETOPMOTOR_DEVICE_ID, MotorType.kBrushless);
    intakebottomMotor = new CANSparkMax(IntakeConstants.INTAKETBOTTOMMOTOR_DEVICE_ID, MotorType.kBrushless);
    intakerotationMotor = new CANSparkMax(IntakeConstants.INTAKEROTATIONMOTOR_DEVICE_ID, MotorType.kBrushless);

    intakerotationMotor.restoreFactoryDefaults();
    m_PidController = intakerotationMotor.getPIDController();
    m_Encoder = intakerotationMotor.getEncoder();

    //PID Coefficients
    kP = 0;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    m_PidController.setP(kP);
    m_PidController.setI(kI);
    m_PidController.setD(kD);
    m_PidController.setIZone(kIz);
    m_PidController.setFF(kFF);
    m_PidController.setOutputRange(kMinOutput, kMaxOutput);

 
    int smartMotionSlot = 0;
    m_PidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_PidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_PidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_PidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

  }

  public void IntakeUp () {

  }
  public void IntakeDown () {

  }
  public void IntakeIn () {
    intaketopMotor.set(1);
    intakebottomMotor.set(1);
  }
  public void IntakeOut (){
    intaketopMotor.set(-1);
    intakebottomMotor.set(-1);
  }
  public void IntakeStop (){
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
