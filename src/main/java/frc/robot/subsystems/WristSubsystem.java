// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
  private TalonFX wristMotor = new TalonFX(Constants.WRIST_MOTOR_ID);
  private TalonFXConfiguration wristConfig = new TalonFXConfiguration();
  private VelocityVoltage wristVelocityVoltage = new VelocityVoltage(0);
  private PositionVoltage wristPositionVoltage = new PositionVoltage(0);
  private double wristSetPosition;

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
    wristConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    wristConfig.Slot0.kP = 0.3;
    wristConfig.Slot0.kI = .005;
    wristConfig.Slot0.kD = 0.0;

    wristMotor.getConfigurator().apply(wristConfig);
    wristMotor.clearStickyFaults();
  }

  public void spinWristMotor(double speed){
    wristMotor.set(speed);
  }

  public void stopWristMotor(){
    wristMotor.stopMotor();
  }

  public double getWristPosition() {
    return wristMotor.getPosition().getValueAsDouble();
  }

  public void updatePosition(){
    wristPositionVoltage.Position = wristMotor.getPosition().getValueAsDouble();
  }

  public void spinByJoystick(DoubleSupplier amount) {
    double spinAmount = MathUtil.applyDeadband(amount.getAsDouble(), .1);
    if(spinAmount>.1||spinAmount<-.01){
      wristVelocityVoltage.Velocity = spinAmount*Constants.WRIST_MAX_ROTATIONS_PER_SEC;
      wristMotor.setControl(wristVelocityVoltage);
      wristSetPosition = wristMotor.getPosition().getValueAsDouble();
    }
  }

  public void wristToIntake(){
    wristPositionVoltage.Position = Constants.WRIST_INTAKE_POSITION;
    wristMotor.setControl(wristPositionVoltage);
    wristSetPosition = Constants.WRIST_INTAKE_POSITION;
  }

  public void wristToL1(){
    wristPositionVoltage.Position = Constants.WRIST_L1_POSITION;
    wristMotor.setControl(wristPositionVoltage);
    wristSetPosition = Constants.WRIST_L1_POSITION;
  }

  public void wristToLMID(){
    wristPositionVoltage.Position = Constants.WRIST_LMID_POSITION;
    wristMotor.setControl(wristPositionVoltage);
    wristSetPosition = Constants.WRIST_LMID_POSITION;
  }

  public void wristToL4(){
    wristPositionVoltage.Position = Constants.WRIST_L4_POSITION;
    wristMotor.setControl(wristPositionVoltage);
    wristSetPosition = Constants.WRIST_L4_POSITION;
  }

  public void wristToBarge() {
    wristPositionVoltage.Position = Constants.WRIST_LOWER_LIMIT+2;
    wristMotor.setControl(wristPositionVoltage);
    wristSetPosition = Constants.WRIST_LOWER_LIMIT+2;
  }

  public Command spinWrist(double speed) {
    return new InstantCommand(()->spinWrist(speed), this);
  }

  public Command stopWrist() {
    return new InstantCommand(()-> stopWristMotor(), this);
  }

  @Override
  public void periodic() {
    if(wristSetPosition > getWristPosition()) {
      wristPositionVoltage.Velocity = .1;
      wristMotor.setControl(wristPositionVoltage);
    }
    else if(wristSetPosition < getWristPosition()) {
      wristPositionVoltage.Velocity = -.1;
      wristMotor.setControl(wristPositionVoltage);
    }
    // This method will be called once per scheduler run

    // updatePosition();
    // wristMotor.setControl(wristPositionVoltage);
  }
}
