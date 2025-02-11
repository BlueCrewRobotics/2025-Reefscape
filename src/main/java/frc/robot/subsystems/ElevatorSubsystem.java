// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;
import java.util.function.DoubleSupplier;

import javax.swing.text.Position;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX motor1 = new TalonFX(Constants.ELEVATOR_MOTOR_LEFT_ID);
  private final TalonFX motor2 = new TalonFX(Constants.ELEVATOR_MOTOR_RIGHT_ID);
  private boolean enableSetPosition = true;
  private double elevatorSetPosition;
  private PositionVoltage elevatorPositionVoltage = new PositionVoltage(0);
  private VelocityVoltage elevatorVelocityVoltage = new VelocityVoltage(0);
  private TalonFXConfiguration climberConfig = new TalonFXConfiguration();
  private CommandXboxController driver;

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
    climberConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motor1.getConfigurator().apply(climberConfig);
    motor2.getConfigurator().apply(climberConfig);
    motor2.setControl(new Follower(Constants.ELEVATOR_MOTOR_LEFT_ID, false));
  }

  public void spinMotor(double speed) {
    motor1.set(speed);
  }

  public void stopMotor() {
    motor1.stopMotor();
    //motor1.setControl(elevatorVelocityVoltage);
    enableSetPosition = false;
  }

  //beginning to try to figure out how to get the elevator to stop in a specific spot
  //have not gotten very far, I would look at Maurader or Fortissimo code as an example 

  public void setHoldPosition(double setPosition) {
    motor1.setPosition(setPosition, .1);
  }

  public void enableSetPosition(boolean enable) {
    enableSetPosition = enable;
  }

  public void setPosition(){
    elevatorPositionVoltage.Position = motor1.getRotorPosition().getValueAsDouble();
    motor1.setControl(elevatorPositionVoltage);
  }

  public double getPosition(){
    return motor1.getPosition().getValueAsDouble();
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

  public void driveByJoystick(DoubleSupplier amount) {
    if(amount.getAsDouble()>.1 || amount.getAsDouble()<-.1){
    double amountSpin = MathUtil.applyDeadband(amount.getAsDouble(), .1);
    elevatorVelocityVoltage.Velocity = amountSpin*Constants.ELEVATOR_MAX_ROTATIONS_PER_SEC;
    motor1.setControl(elevatorVelocityVoltage);
    enableSetPosition = false;
    elevatorSetPosition = getPosition();
    }
    else {
    enableSetPosition = true;
    }
  }

  public void addPosition(CommandXboxController driver){
    this.driver = driver;
    if(driver.povUp().getAsBoolean()){
    elevatorPositionVoltage.Position += 1;
    motor1.setControl(elevatorPositionVoltage);
    }
  }

  public void L2Reef() {
    elevatorPositionVoltage.Position = Constants.L2REEFPOSITION;
    motor1.setControl(elevatorPositionVoltage);
    enableSetPosition = true;
    elevatorSetPosition = Constants.L2REEFPOSITION;
  }

  public void intakeCoral(){
    elevatorPositionVoltage.Position = Constants.CORALSTATION;
    motor1.setControl(elevatorPositionVoltage);
    enableSetPosition = true;
    elevatorSetPosition = Constants.CORALSTATION;
  }

  public void returnHome(){
    elevatorPositionVoltage.Position = Constants.ELEVATOR_LOWER_LIMIT;
  }
  
  @Override
  public void periodic() {
      // if (elevatorSetPosition > getPosition()) {
      //   elevatorPositionVoltage.Velocity = .2;
      //   motor1.setControl(elevatorPositionVoltage);
      // }
      // else if (elevatorSetPosition < getPosition()) {
      //   elevatorPositionVoltage.Velocity = -.2;
      //   motor1.setControl(elevatorPositionVoltage);
      // }
    // }
    // if(!enableSetPosition){
    //   if (elevatorSetPosition > getPosition()) {
    //     elevatorVelocityVoltage.Velocity = -.1;
    //   }
    //   else if (elevatorSetPosition < getPosition()) {
    //     elevatorVelocityVoltage.Velocity = .1;
    //   }
    // }
  }
}
