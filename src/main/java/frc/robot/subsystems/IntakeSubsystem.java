// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_ID);
  private final TalonFXConfiguration intakeConfig= new TalonFXConfiguration();
  /** Creates a new IntakeModule. */
  public IntakeSubsystem() {
    intakeConfig.CurrentLimits.SupplyCurrentLimit = 60;
    intakeConfig.CurrentLimits.SupplyCurrentLowerLimit = 35;
    intakeConfig.CurrentLimits.SupplyCurrentLowerTime = .1;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeMotor.clearStickyFaults();
  }

  public boolean coralInIntake() {
    if (intakeMotor.getSupplyVoltage().getValueAsDouble() > 20) {
      return true;
    }
    else return false;
  }

  public void spinMotor(double speed) {
    intakeMotor.set(speed);
  }

  public void stopMotor() {
    intakeMotor.stopMotor();
  }

  public Command intakeAlgae() {
    return this.run(
      () -> spinMotor(.3)
    );
  }

  public Command stopIntake() {
    return new InstantCommand(() -> stopMotor(), this);
  }

  public Command intakeCoral() {
    return new InstantCommand(()->spinMotor(-.3), this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Motor Temperature", intakeMotor.getDeviceTemp().getValueAsDouble());
  }
}
