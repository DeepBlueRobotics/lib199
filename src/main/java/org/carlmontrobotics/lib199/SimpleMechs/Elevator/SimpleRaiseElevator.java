// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.lib199.SimpleMechs.Elevator;

import org.carlmontrobotics.lib199.SimpleMechs.Elevator.SimpleElevator.ElevatorControlState;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SimpleRaiseElevator extends Command {
  private SimpleElevator elevator;
  private double elevatorGoal;
  private boolean useMechanicalLimit;
  private boolean commandComplete = false;
  private double safeMechanicalUsageSpeed = 0.1;

  /** Creates a new SimpleRaiseElevator. */
  public SimpleRaiseElevator(SimpleElevator elevator, double height, boolean useMechanicalLimit) {
    addRequirements(this.elevator = elevator);
    this.elevatorGoal = height;
    this.useMechanicalLimit = useMechanicalLimit;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setGoal(elevatorGoal);
    elevator.setAutoOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevator.atGoal()) {
      if (elevator.getConfig().bottomLimit == elevatorGoal && useMechanicalLimit) {
        if (elevator.getElevatorHeight() <= elevatorGoal) {commandComplete = true;}
        if (elevator.getCurrentControlState() != ElevatorControlState.MANUAL) {
          elevator.setManualOn();
        }
        elevator.setManualInput(-safeMechanicalUsageSpeed);
      }
      else if (elevator.getConfig().topLimit == elevatorGoal && useMechanicalLimit) {
        if (elevator.getElevatorHeight() >= elevatorGoal) {commandComplete = true;}
        if (elevator.getCurrentControlState() != ElevatorControlState.MANUAL) {
          elevator.setManualOn();
        }
        elevator.setManualInput(safeMechanicalUsageSpeed);
      }
      else {
        commandComplete = true;
      }
    }
    else {
      if (elevator.getCurrentControlState() != ElevatorControlState.AUTO) {
        elevator.setAutoOn();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setAutoOn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return commandComplete;
  }
}
