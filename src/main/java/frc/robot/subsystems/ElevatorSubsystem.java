// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;
import java.util.function.DoubleSupplier;

import javax.swing.text.Position;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private Servo elevatorStopper;
    private double elevatorStopperPosition;
    private final CANcoder elevatorCANcoder;
  
    /** Creates a new Elevator. */
    public ElevatorSubsystem() {
      motor1.clearStickyFaults();
      motor2.clearStickyFaults();
  
      climberConfig.CurrentLimits.SupplyCurrentLimit = 40;
      climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  
      climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ELEVATOR_UPPER_LIMIT;
      climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.ELEVATOR_LOWER_LIMIT;    
      climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
  
      climberConfig.Slot0.kP = 0.435;
      climberConfig.Slot0.kI = 0.02;
      climberConfig.Slot0.kD = 0.0;
      climberConfig.Slot0.kG = 0.65;
  
      climberConfig.Slot1.kP = 0.18;
      climberConfig.Slot1.kI = 0;//0.005;
      climberConfig.Slot1.kD = 0.0;
      climberConfig.Slot1.kG = 0.25;
      climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      climberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      elevatorCANcoder = new CANcoder(Constants.ELEVATOR_CAN_CODER_ID);

    motor1.getConfigurator().apply(climberConfig);
    motor2.getConfigurator().apply(climberConfig);
    motor2.setControl(new Follower(Constants.ELEVATOR_MOTOR_LEFT_ID, false));
    resetMotorEncoderToAbsolute();
    elevatorStopper = new Servo(0);
  }

  public void spinMotor(double speed) {
    motor1.set(speed);
  }

  public void stopMotor() {
    motor1.stopMotor();
    enableSetPosition = false;
  }

  public void setHoldPosition(double setPosition) {
    motor1.setPosition(setPosition, .1);
  }

  public double getPosition(){
    return motor1.getPosition().getValueAsDouble();
  }

  public Command stopElevator() {
    enableSetPosition = true;
    return new InstantCommand(()->stopMotor(), this);
  }

  public Command resetPosition() {
    return this.run(()->motor1.setPosition(Constants.ELEVATOR_LOWER_LIMIT)).andThen(this.run(()->elevatorSetPosition = Constants.ELEVATOR_LOWER_LIMIT));
  }

  //lets the driver control the position, when no input is given the elevator will hold its position
  public void driveByJoystick(DoubleSupplier amount) {
    if(amount.getAsDouble()>.1 || amount.getAsDouble()<-.1){
      double amountSpin = MathUtil.applyDeadband(amount.getAsDouble(), .1);
      elevatorVelocityVoltage.Velocity = -amountSpin*Constants.ELEVATOR_MAX_ROTATIONS_PER_SEC;
      motor1.setControl(elevatorVelocityVoltage);
      elevatorSetPosition = getPosition();
    }
    else {
      if (motor1.getPosition().getValueAsDouble() <= elevatorSetPosition) {
        motor1.setControl(elevatorPositionVoltage.withPosition(elevatorSetPosition).withSlot(0));
      }
      else {
        motor1.setControl(elevatorPositionVoltage.withPosition(elevatorSetPosition).withSlot(1));
      }
    }
  }

  //gives new set positions for the elevator to go to 
  public Command L1Reef() {
    return this.runOnce(() -> elevatorSetPosition = Constants.L1REEFPOSITION);
  }

  public Command L2Reef() {
    return this.runOnce(() -> elevatorSetPosition = Constants.L2REEFPOSITION);
  }

  public Command L3Reef() {
    return this.runOnce(() -> elevatorSetPosition = Constants.L3REEFPOSITION);
  }

  public Command L4Reef() {
    return this.runOnce(() -> elevatorSetPosition = Constants.ELEVATOR_UPPER_LIMIT);
  }

  public Command intakeCoral(){
    return this.runOnce(() -> elevatorSetPosition = Constants.CORALSTATION);
  }

  public Command returnHome(){
    return this.runOnce(() -> elevatorSetPosition = Constants.ELEVATOR_LOWER_LIMIT);
  }

  public Command linearActuatorOut(){
    return this.run(
      () -> elevatorStopper.set(1)
      );
  }

  public Command linearActuatorIn() {
    return this.run(
      () -> elevatorStopper.set(200d / 370d)
      );
  }
    
  
  public Angle getElevatorPosition() {
    return elevatorCANcoder.getAbsolutePosition().getValue() ;
}
public void resetMotorEncoderToAbsolute() {
  Angle newPosition = getElevatorPosition() ;

  motor1.setPosition(newPosition);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //makes sure when the robot is disabled the position is being updated so when reenabled the elevator will not
    //go back to the position it was at when it was disabled (hazard)
    if (RobotState.isDisabled()) {
      elevatorSetPosition = getPosition();
    }

    elevatorStopperPosition = elevatorStopper.getPosition();
    SmartDashboard.putNumber("Servo Position", elevatorStopperPosition);
  }
}
