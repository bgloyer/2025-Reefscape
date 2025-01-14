// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SparkFlex m_motor;

  public AlgaeIntake() {
    m_motor = new SparkFlex(11, MotorType.kBrushless);
  }

  /** @speed negative is intake */
  public void runMotor(double speed) {
    m_motor.set(speed);
  }

  public void stopMotor() {
    m_motor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
