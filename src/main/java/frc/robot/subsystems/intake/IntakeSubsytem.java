
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsytem extends SubsystemBase {
 
  private CANSparkMax intakeinMotor;
  private CANSparkMax intakerotationMotor;
  
  private SparkPIDController m_PidController;
  private RelativeEncoder m_Encoder;
  public double kP, kI, kD, kIz, kMaxOutput, kFF, setPositionHigh, setPositionLow;
  public double kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;


  /** Creates a new ShooterSubsytem. */
  public IntakeSubsytem() {
    intakeinMotor = new CANSparkMax(IntakeConstants.INTAKEINMOTOR_DEVICE_ID, MotorType.kBrushless);
    intakerotationMotor = new CANSparkMax(IntakeConstants.INTAKEROTATIONMOTOR_DEVICE_ID, MotorType.kBrushless);

    intakerotationMotor.restoreFactoryDefaults();
    m_PidController = intakerotationMotor.getPIDController();
    m_Encoder = intakerotationMotor.getEncoder();

    //PID Coefficients
    kP = 0.04;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    setPositionLow = 0;
    setPositionHigh = -8.5;

    m_PidController.setP(kP);
    m_PidController.setI(kI);
    m_PidController.setD(kD);
    m_PidController.setIZone(kIz);
    m_PidController.setFF(kFF);
    m_PidController.setOutputRange(kMinOutput, kMaxOutput);


  }

  public void IntakeUp () {
    m_PidController.setReference(setPositionHigh, CANSparkMax.ControlType.kPosition);

  }
  public void IntakeDown () {
    m_PidController.setReference(setPositionLow, CANSparkMax.ControlType.kPosition);
  }
  public void IntakeIn () {
    intakeinMotor.set(-1);
  }
  public void IntakeOut (){
    intakeinMotor.set(1);
  }
  public void IntakeStop (){
    intakeinMotor.set(0);
  }

  @Override
  public void periodic() {

  }
}
  

