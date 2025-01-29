// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX motor1 = new TalonFX(Constants.ELEVATOR_MOTOR_LEFT_ID);
  private final TalonFX motor2 = new TalonFX(Constants.ELEVATOR_MOTOR_RIGHT_ID);
  private boolean enableSetPosition;

  private TalonFXConfiguration climberConfig = new TalonFXConfiguration();

  /** Creates a new Elevator. */
  public ElevatorSubsystem() {
    motor1.clearStickyFaults();
    motor2.clearStickyFaults();

//    climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
//
//    climberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    climberConfig.CurrentLimits.SupplyCurrentLimit = 40;
    //climberConfig.CurrentLimits. = 60;
    climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ELEVATOR_UPPER_LIMIT;
climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.ELEVATOR_LOWER_LIMIT;    
climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    climberConfig.Slot0.kP = 0.3;
    climberConfig.Slot0.kI = 0.005;
    climberConfig.Slot0.kD = 0.0;
    climberConfig.Slot0.kG = 0.3;

    climberConfig.Slot1.kP = 0.3;
    climberConfig.Slot1.kI = 0.005;
    climberConfig.Slot1.kD = 0.0;
    climberConfig.Slot1.kG = 0.25;
    climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motor1.getConfigurator().apply(climberConfig);
    motor2.getConfigurator().apply(climberConfig);
    motor2.setControl(new Follower(Constants.ELEVATOR_MOTOR_LEFT_ID, false));
  }

  public void spinMotor(double speed) {
    motor1.set(speed);
  }

  public void stopMotor() {
    motor1.stopMotor();
  }

  //beginning to try to figure out how to get the elevator to stop in a specific spot
  //have not gotten very far, I would look at Maurader or Fortissimo code as an example 

  public void setHoldPosition(double setPosition) {
    motor1.setPosition(setPosition, .1);
  }

  public void enableSetPosition(boolean enable) {
    enableSetPosition = enable;
  }

  public Command goUp(double speed) {
    return new InstantCommand(()->spinMotor(-speed),this);
  }
  public Command goDown(double speed) {
    return new InstantCommand(()->spinMotor(speed),this);
  }

  public Command stopElevator() {
    enableSetPosition = true;
    return new InstantCommand(()->stopMotor(), this);
  }

  public Command driveByJoystick(double amount) {
    return new InstantCommand(() -> spinMotor(amount/10), this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
