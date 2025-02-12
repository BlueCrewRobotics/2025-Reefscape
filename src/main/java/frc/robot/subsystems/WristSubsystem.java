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
import edu.wpi.first.wpilibj.RobotState;
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
    wristConfig.Slot0.kG = .3;

    wristConfig.Slot1.kP =0.0;
    wristConfig.Slot1.kI = 0.0;
    wristConfig.Slot1.kD = 0.0;
    wristConfig.Slot1.kG = .3;

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
      wristSetPosition = getWristPosition();
    }
    else {
      wristMotor.setControl(wristPositionVoltage.withPosition(wristSetPosition).withSlot(0));
    }
  }

  public Command wristToIntake(){
    return this.run(() -> wristSetPosition = Constants.WRIST_INTAKE_POSITION);
  }

  public Command wristToL1(){
    return this.run(() -> wristSetPosition = Constants.WRIST_L1_POSITION);
  }

  public Command wristToLMID(){
    return this.run(() -> wristSetPosition = Constants.WRIST_LMID_POSITION);
  }

  public Command wristToL4(){
    return this.run(() -> wristSetPosition = Constants.WRIST_L4_POSITION);
  }

  public Command wristToBarge() {
    return this.run(() -> wristSetPosition = Constants.WIRST_BARGE_POSITION);
  }

  public Command spinWrist(double speed) {
    return new InstantCommand(()->spinWrist(speed), this);
  }

  public Command stopWrist() {
    return new InstantCommand(()-> stopWristMotor(), this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(RobotState.isDisabled()) {
      wristSetPosition = getWristPosition();
    }
  }
}
