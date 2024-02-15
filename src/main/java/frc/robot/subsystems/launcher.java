// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class launcher extends SubsystemBase {
  private int LSHOOTER = 20;
  private int RSHOOTER = 21;
  public final int maxRPM = 5700;
  /** Creates a new shooter. */
  CANSparkMax leftLauncherSparkMax = new CANSparkMax(LSHOOTER, MotorType.kBrushless);
  CANSparkMax rightLauncherSparkMax = new CANSparkMax(RSHOOTER, MotorType.kBrushless);
  
  SparkPIDController rightLauncherPIDController = rightLauncherSparkMax.getPIDController();
  SparkPIDController leftLauncerPIDController = leftLauncherSparkMax.getPIDController();

  public launcher() {
    rightLauncherSparkMax.restoreFactoryDefaults();
    rightLauncherSparkMax.setIdleMode(IdleMode.kCoast);
    rightLauncherSparkMax.setInverted(true);
    rightLauncherSparkMax.setSmartCurrentLimit(40);
    rightLauncherPIDController.setP(.1);
    rightLauncherPIDController.setI(0);
    rightLauncherPIDController.setD(0);
    rightLauncherPIDController.setFF(0);

    leftLauncherSparkMax.restoreFactoryDefaults();
    leftLauncherSparkMax.setIdleMode(IdleMode.kCoast);
    leftLauncherSparkMax.setInverted(false);
    leftLauncherSparkMax.setSmartCurrentLimit(40);
    leftLauncerPIDController.setP(.1);
    leftLauncerPIDController.setI(0);
    leftLauncerPIDController.setD(0);
    leftLauncerPIDController.setFF(0);  
  }

public void SetDutyOutlaunch(double percent) {
  rightLauncherSparkMax.set(percent);
  leftLauncherSparkMax.set(percent);
}

public double getRightLauncherRPM() {
 return rightLauncherSparkMax.getEncoder().getVelocity();
}

public double getLeftLauncherRPM() {
 return leftLauncherSparkMax.getEncoder().getVelocity();
}

public void SetLauncherVelocity(double setpoint) {
    rightLauncherPIDController.setReference(setpoint, ControlType.kVelocity);
    leftLauncerPIDController.setReference(setpoint, ControlType.kVelocity);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Right launcher RPM", getRightLauncherRPM());
    SmartDashboard.putNumber("Left launcher RPM", getLeftLauncherRPM());
  }
}
