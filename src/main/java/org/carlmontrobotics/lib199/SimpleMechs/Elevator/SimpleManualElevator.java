// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.lib199.SimpleMechs.Elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SimpleManualElevator extends Command {
  private SimpleElevator elevator;
  private DoubleSupplier manualInput;
  private boolean remainAtGoal;
  /** Creates a new SimpleManualElevator. */
  public SimpleManualElevator(SimpleElevator elevator, DoubleSupplier manualInput, boolean remainAtGoal) {
    addRequirements(this.elevator = elevator);
    this.manualInput = manualInput;
    this.remainAtGoal = remainAtGoal;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setManualOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setManualInput(manualInput.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (remainAtGoal) {
      elevator.setGoal(elevator.getElevatorHeight());
      elevator.setAutoOn();
    }
    else {
      elevator.setBrakeOn();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
