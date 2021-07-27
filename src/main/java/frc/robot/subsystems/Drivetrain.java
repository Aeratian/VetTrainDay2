// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  public TalonFX leftLead;
  public TalonFX leftFollow;
  public TalonFX rightLead;
  public TalonFX rightFollow;
  private AHRS gyro;

  public static final double WHEEL_DIAMETER_METERS = .158;
  public static final double TICKS_PER_ROTATION = 2048;
  public static final double GEAR_RATIO = 11.2444;

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    leftLead = new TalonFX(2);
    leftFollow = new TalonFX(1);
    rightLead = new TalonFX(3);
    rightFollow = new TalonFX(5);
    gyro = new AHRS(SPI.Port.kMXP);

    rightFollow.follow(rightLead);
    leftFollow.follow(rightLead);

    rightLead.setInverted(true);
    rightFollow.setInverted(true);
  }

  public double getAngle() {
    return gyro.getAngle();
  }

  public double getRightDistance() {
    double distanceTicks = (rightLead.getSelectedSensorPosition() + rightFollow.getSelectedSensorPosition()) / 2;
    return ticksToMeters(distanceTicks);
  }

  public double getLeftDistance() {
    double distanceTicks = (leftLead.getSelectedSensorPosition() + leftFollow.getSelectedSensorPosition()) / 2;
    return ticksToMeters(distanceTicks);
  }

  public double ticksToMeters(double ticks) {
    return ticks / TICKS_PER_ROTATION * WHEEL_DIAMETER_METERS * Math.PI / GEAR_RATIO;
  }

  /**
   * Runs the motors
   * @param left  percent output to the left side [-1 to 1]
   * @param right percent output to the right side [-1 to 1]
   */
  public void runMotors(double left, double right) {
    rightLead.set(ControlMode.PercentOutput, left);
    leftLead.set(ControlMode.PercentOutput, right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // log data to shuffleboard for debugging
    SmartDashboard.putNumber("Right Output", rightLead.getMotorOutputPercent());
    SmartDashboard.putNumber("Left Output", leftLead.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Distance", getRightDistance());
    SmartDashboard.putNumber("Left Distance", getLeftDistance());
  }
}
