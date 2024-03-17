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
  public final int PODIUM_TARGET_RPM_LEFT = 5000;
  public final int PODIUM_TARGET_RPM_RIGHT = 3000;
  public final int SPEAKER_TARGET_RPM_LEFT = 4500;
  public final int SPEAKER_TARGET_RPM_RIGHT = 2700;
  public final int AMP_TARGET_RPM_LEFT = 2500;
  public final int AMP_TARGET_RPM_RIGHT = 2000; 


  /** Creates a new shooter. */
  CANSparkMax leftLauncherSparkMax = new CANSparkMax(LSHOOTER, MotorType.kBrushless);
  CANSparkMax rightLauncherSparkMax = new CANSparkMax(RSHOOTER, MotorType.kBrushless);
  
  SparkPIDController rightLauncherPIDController = rightLauncherSparkMax.getPIDController();
  SparkPIDController leftLauncerPIDController = leftLauncherSparkMax.getPIDController();

  public launcher() {
    rightLauncherSparkMax.restoreFactoryDefaults();
    rightLauncherSparkMax.setIdleMode(IdleMode.kBrake);
    rightLauncherSparkMax.setInverted(true);
    rightLauncherSparkMax.setSmartCurrentLimit(50);
    rightLauncherSparkMax.setOpenLoopRampRate(0.1);
    rightLauncherSparkMax.setClosedLoopRampRate(0.1);
    rightLauncherPIDController.setP(1.5e-4);
    rightLauncherPIDController.setI(0);
    rightLauncherPIDController.setD(0);
    rightLauncherPIDController.setFF(0.00017);

    leftLauncherSparkMax.restoreFactoryDefaults();
    leftLauncherSparkMax.setIdleMode(IdleMode.kBrake);
    leftLauncherSparkMax.setInverted(false);
    leftLauncherSparkMax.setSmartCurrentLimit(50);
    leftLauncherSparkMax.setOpenLoopRampRate(0.1);
    leftLauncherSparkMax.setClosedLoopRampRate(0.1);
    leftLauncerPIDController.setP(1.5e-4);
    leftLauncerPIDController.setI(0);
    leftLauncerPIDController.setD(0);
    leftLauncerPIDController.setFF(0.00017);  
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

public void SetLauncherVelocity(double right, double left) {
    rightLauncherPIDController.setReference(right, ControlType.kVelocity);
    leftLauncerPIDController.setReference(left, ControlType.kVelocity);
}
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Launcher/Right RPM", getRightLauncherRPM());
    SmartDashboard.putNumber("Launcher/Right stator current", rightLauncherSparkMax.getOutputCurrent());
    SmartDashboard.putNumber("Launcher/Left RPM", getLeftLauncherRPM());
    SmartDashboard.putNumber("Launcher/Left stator current", leftLauncherSparkMax.getOutputCurrent());
  }
}
