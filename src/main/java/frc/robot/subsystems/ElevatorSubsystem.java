// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX motor1 = new TalonFX(Constants.ELEVATOR_MOTOR_LEFT_ID);
  private final TalonFX motor2 = new TalonFX(Constants.ELEVATOR_MOTOR_RIGHT_ID);

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

    motor1.getConfigurator().apply(climberConfig);
    motor2.getConfigurator().apply(climberConfig);
    motor2.setControl(new Follower(Constants.ELEVATOR_MOTOR_LEFT_ID, false));

    motor1.clearStickyFaults();
    motor2.clearStickyFaults();
  }

  public void spinMotor(double speed) {
    motor1.set(speed);
  }

  public void stopMotor() {
    motor1.stopMotor();
  }

  public Command goUp() {
    return new InstantCommand(()->spinMotor(10),this);
  }
  public Command goDown() {
    return new InstantCommand(()->spinMotor(-10),this);
  }

  public Command stopElevator() {
    return new InstantCommand(()->stopMotor(), this);
  }

  public Command driveByJoystick(double amount) {
    return new InstantCommand(() -> spinMotor(amount), this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
