/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private Joystick m_stick;
  private static final int deviceID = 1;
  private CANSparkMax m_Motor1;
  private CANSparkMax m_Motor2;
  private CANPIDController m_pidController1;
  private CANPIDController m_pidController2;
  private CANEncoder m_encoder1;
  private CANEncoder m_encoder2;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, RPM;

  @Override
  public void robotInit() {
    m_stick = new Joystick(0);

    // initialize motor
    m_Motor1 = new CANSparkMax(1, MotorType.kBrushless);
    m_Motor2 = new CANSparkMax(2, MotorType.kBrushless);

    RPM = 5676;


    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_Motor1.restoreFactoryDefaults();
    m_Motor2.restoreFactoryDefaults();

    /**
     * In order to use PID functionality for a controller, a CANPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController1 = m_Motor1.getPIDController();
    m_pidController2 = m_Motor2.getPIDController();

    // Encoder object created to display position values
    m_encoder1 = m_Motor1.getEncoder();
    m_encoder2 = m_Motor2.getEncoder();

    // PID coefficients
    kP = 6e-4;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.000015;
    kMaxOutput = 5700;
    kMinOutput = -5700;
    maxRPM = 5700;

    // set PID coefficients
    m_pidController1.setP(kP);
    m_pidController1.setI(kI);
    m_pidController1.setD(kD);
    m_pidController1.setIZone(kIz);
    m_pidController1.setFF(kFF);
    m_pidController1.setOutputRange(kMinOutput, kMaxOutput);

    m_pidController2.setP(kP);
    m_pidController2.setI(kI);
    m_pidController2.setD(kD);
    m_pidController2.setIZone(kIz);
    m_pidController2.setFF(kFF);
    m_pidController2.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("RPM demand", RPM);
  }

  @Override
  public void teleopPeriodic() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double RPM = SmartDashboard.getNumber("RPM", 0);
    

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController1.setP(p); kP = p; }
    if((i != kI)) { m_pidController1.setI(i); kI = i; }
    if((d != kD)) { m_pidController1.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController1.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController1.setFF(ff); kFF = ff; }
    if((RPM != RPM)) { RPM = RPM; }
    if((max != kMaxOutput) || (min != kMinOutput)) {
      m_pidController1.setOutputRange(min, max);
      kMinOutput = min; kMaxOutput = max;
    }

    if((p != kP)) { m_pidController2.setP(p); kP = p; }
    if((i != kI)) { m_pidController2.setI(i); kI = i; }
    if((d != kD)) { m_pidController2.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController2.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController2.setFF(ff); kFF = ff; }
    if((RPM != RPM)) { RPM = RPM; }
    if((max != kMaxOutput) || (min != kMinOutput)) {
      m_pidController2.setOutputRange(min, max);
      kMinOutput = min; kMaxOutput = max;
    }

    /**
     * PIDController objects are commanded to a set point using the
     * SetReference() method.
     *
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     *
     * The second parameter is the control type can be set to one of four
     * parameters:
     *  com.revrobotics.ControlType.kDutyCycle
     *  com.revrobotics.ControlType.kPosition
     *  com.revrobotics.ControlType.kVelocity
     *  com.revrobotics.ControlType.kVoltage
     */

    m_pidController1.setReference(-1140, ControlType.kVelocity);
    m_pidController2.setReference(1140, ControlType.kVelocity);

    SmartDashboard.putNumber("ProcessVariable", m_encoder1.getVelocity());
    SmartDashboard.putNumber("ProcessVariable", m_encoder2.getVelocity());
  }
}
