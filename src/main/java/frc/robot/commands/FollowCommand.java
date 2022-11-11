// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Util;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.RomiDrivetrain;

public class FollowCommand extends CommandBase {
  private RomiDrivetrain drivetrain;
  private PhotonVision photonVision;

  /** Creates a new FollowCommand. */
  public FollowCommand(RomiDrivetrain drivetrain, PhotonVision camera) {
    this.drivetrain = drivetrain;
    this.photonVision = camera;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult result = photonVision.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();

      double area = target.getArea();
      double speed = 0;
      if (area >= Constants.backupArea) {
        speed = Constants.backupSpeed;
      } else if (area <= Constants.maxArea) {
        speed = Util.map(target.getArea(), Constants.minArea, Constants.maxArea, Constants.maxSpeed, Constants.minSpeed);
      }

      double turn = Util.map(target.getYaw(), Constants.minYaw, Constants.maxYaw, Constants.minTurn, Constants.maxTurn);
      
      drivetrain.arcadeDrive(speed, turn);
    } else {
      drivetrain.arcadeDrive(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
