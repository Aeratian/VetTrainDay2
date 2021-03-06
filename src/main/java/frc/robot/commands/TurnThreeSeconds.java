// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TurnThreeSeconds extends CommandBase {

  Drivetrain drivetrain;
  double startTime;

  /** Creates a new TurnThreeSeconds. */
  public TurnThreeSeconds(Drivetrain _drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = _drivetrain;
    startTime = -1;
    addRequirements(drivetrain); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.runMotors(0.5, -0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.runMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime > 3;
  }
}
