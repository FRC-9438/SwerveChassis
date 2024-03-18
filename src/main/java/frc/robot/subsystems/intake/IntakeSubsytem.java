
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsytem extends SubsystemBase {
 
  private CANSparkMax intaketopMotor;
  private CANSparkMax intakebottomMotor;
  private CANSparkMax intakerotationMotor;
  
  private SparkPIDController m_PidController;
  private RelativeEncoder m_Encoder;
  public double kP, kI, kD, kIz, kMaxOutput, kFF, setPositionHigh, setPositionLow;
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

    setPositionLow = 0;
    setPositionHigh = 50;

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
    m_PidController.setReference(setPositionHigh, CANSparkMax.ControlType.kSmartMotion);

  }
  public void IntakeDown () {
    m_PidController.setReference(setPositionLow, CANSparkMax.ControlType.kSmartMotion);
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
      double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    double minV = SmartDashboard.getNumber("Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_PidController.setP(p); kP = p; }
    if((i != kI)) { m_PidController.setI(i); kI = i; }
    if((d != kD)) { m_PidController.setD(d); kD = d; }
    if((iz != kIz)) { m_PidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_PidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_PidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel)) { m_PidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { m_PidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { m_PidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { m_PidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }

    double setPoint, processVariable;
    boolean mode = SmartDashboard.getBoolean("Mode", false);
    if(mode) {
      setPoint = SmartDashboard.getNumber("Set Velocity", 0);
      m_PidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
      processVariable = m_Encoder.getVelocity();
    } else {
      setPoint = SmartDashboard.getNumber("Set Position", 0);
      /**
       * As with other PID modes, Smart Motion is set by calling the
       * setReference method on an existing pid object and setting
       * the control type to kSmartMotion
       */
      m_PidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
      processVariable = m_Encoder.getPosition();
    }
    
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("Process Variable", processVariable);
    SmartDashboard.putNumber("Output", intakerotationMotor.getAppliedOutput());
  }
}
  

