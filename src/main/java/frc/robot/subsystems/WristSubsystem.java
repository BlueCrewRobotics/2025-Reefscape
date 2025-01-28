// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
  private TalonFX wristMotor = new TalonFX(Constants.WRIST_MOTOR_ID);
  private TalonFXConfiguration wristConfig = new TalonFXConfiguration();

  /** Creates a new WristModule. */
  public WristSubsystem() {
    wristConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.WRIST_UPPER_LIMIT;
    wristConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.WRIST_LOWER_LIMIT;
    wristConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    wristConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    wristConfig.CurrentLimits.SupplyCurrentLimit = 60;
    wristConfig.CurrentLimits.SupplyCurrentLowerLimit = 35;
    wristConfig.CurrentLimits.SupplyCurrentLowerTime = .01;
    wristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    wristMotor.getConfigurator().apply(wristConfig);
    wristMotor.clearStickyFaults();

  }

  public void setWristPosition(double position){
    wristMotor.setPosition(position);
  }

  public void spinWrist(double speed){
    wristMotor.set(speed);
  }

  public void stopWrist(){
    wristMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
