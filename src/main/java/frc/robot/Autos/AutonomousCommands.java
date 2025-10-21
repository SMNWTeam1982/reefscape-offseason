package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem.ElevatorConstants;
import frc.robot.subsystems.Wrist.CoralSubsystem;
import frc.robot.subsystems.Wrist.WristSubsystem;
import frc.robot.subsystems.Wrist.WristSubsystem.WristConstants;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.swerve.ReefNavigation;

public final class AutonomousCommands {

    /**
     * creates a new moveToPose command with a new target every time this command is ititialized
     * <p> the target pose wont change while the command is running but will change on init
     */
    public static Command navigateToNearestScoringPose(DriveSubsystem drive) {
        return drive.defer(() -> drive.moveToPose(ReefNavigation.getClosestScoringPose(drive.getEstimatedPose())));
    }

    /**
     * this command requires the drive subsystem and the elevator subsystem, meaning their default
     * commands will be cancelled
     */
    public static Command scoreNearestL1(
            DriveSubsystem drive, ElevatorSubsystem elevator, WristSubsystem wrist, CoralSubsystem intake) {
        return navigateToNearestScoringPose(drive)
                .andThen(elevator.holdHeight(ElevatorConstants.LEVEL_1_TARGET_HEIGHT)
                        .alongWith(wrist.holdAngle(WristConstants.LEVEL_1_POSITION)))
                .until(elevator.atTargetHeight.and(wrist.atTargetAngle))
                .andThen(intake.eject()
                        .alongWith(elevator.holdHeight(ElevatorConstants.LEVEL_1_TARGET_HEIGHT))
                        .alongWith(wrist.holdAngle(WristConstants.LEVEL_1_POSITION))
                        .withTimeout(1))
                .andThen(elevator.setIdle(), wrist.setIdle());
    }

    public static Command scoreNearestL2(
            DriveSubsystem drive, ElevatorSubsystem elevator, WristSubsystem wrist, CoralSubsystem intake) {
        return navigateToNearestScoringPose(drive)
                .andThen(elevator.holdHeight(ElevatorConstants.LEVEL_2_TARGET_HEIGHT)
                        .alongWith(wrist.holdAngle(WristConstants.LEVEL_MID_POSITION)))
                .until(elevator.atTargetHeight.and(wrist.atTargetAngle))
                .andThen(intake.eject()
                        .alongWith(elevator.holdHeight(ElevatorConstants.LEVEL_2_TARGET_HEIGHT))
                        .alongWith(wrist.holdAngle(WristConstants.LEVEL_MID_POSITION))
                        .withTimeout(1))
                .andThen(elevator.setIdle(), wrist.setIdle());
    }

    public static Command scoreNearestL3(
            DriveSubsystem drive, ElevatorSubsystem elevator, WristSubsystem wrist, CoralSubsystem intake) {
        return navigateToNearestScoringPose(drive)
                .andThen(elevator.holdHeight(ElevatorConstants.LEVEL_3_TARGET_HEIGHT) // Moves elevator to L3 Pos
                        .alongWith(wrist.holdAngle(WristConstants.LEVEL_MID_POSITION))) // Moves wrist to mid angle
                .until(elevator.atTargetHeight.and(
                        wrist.atTargetAngle)) // Waits until the wrist and elevaotr are at the required pos
                .andThen(
                        intake.eject() // ejects coral
                                .alongWith(elevator.holdHeight(ElevatorConstants.LEVEL_3_TARGET_HEIGHT))
                                .alongWith(wrist.holdAngle(WristConstants.LEVEL_MID_POSITION))
                                .withTimeout(1)) // ^ Holds height/angle for 1 second ^
                .andThen(elevator.setIdle(), wrist.setIdle()); // returns to idle position
    }

    public static command stationtocoral(DriveSubsystem drive) {
        return navigateToNearestScoringPose(drive);
    }
}
