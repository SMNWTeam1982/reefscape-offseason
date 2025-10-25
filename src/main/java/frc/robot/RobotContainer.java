// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Autos.AutonomousCommands;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Algae.AlgaeSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem.ElevatorConstants;
import frc.robot.subsystems.Wrist.CoralSubsystem;
import frc.robot.subsystems.Wrist.WristSubsystem;
import frc.robot.subsystems.Wrist.WristSubsystem.WristConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;
import frc.robot.subsystems.vision.PhotonVisionSubsystem.PhotonVisionConstants;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final PhotonVisionSubsystem visionSubsystem = new PhotonVisionSubsystem(
            PhotonVisionConstants.PHOTON_CAM_RELATIVE_TO_ROBOT, PhotonVisionConstants.PHOTON_CAMERA_NAME);

    private final DriveSubsystem driveSubsystem =
            new DriveSubsystem(() -> visionSubsystem.getLastVisionData(), () -> visionSubsystem.isDataFresh());

    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    private final WristSubsystem wristSubsystem = new WristSubsystem();

    private final CoralSubsystem coralSubsystem = new CoralSubsystem();

    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();

    private final LoggedDashboardChooser<Command> autoChooser;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandJoystick operatorController = new CommandJoystick(OperatorConstants.OPERATOR_CONTROLLER_PORT);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    @SuppressWarnings("unused")
    public RobotContainer() {
        autoChooser = new LoggedDashboardChooser<>("Selected Auto Routine");
        autoChooser.addOption(
                "nearest L2",
                AutonomousCommands.scoreNearestL2(driveSubsystem, elevatorSubsystem, wristSubsystem, coralSubsystem));
        autoChooser.addDefaultOption("nothing", Commands.none());

        // Configure the trigger bindings
        configureDriverBindings();
        configureOperatorBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     *
     * <p>see the wpilib page on coordinates and joystick coordinates
     */
    private void configureDriverBindings() {
        // var alliance = DriverStation.getAlliance();
        // boolean onBlueSide = true; // default to blue mode
        // if (alliance.isPresent()) {
        //     onBlueSide = alliance.get() != Alliance.Blue; // had to reverse this due to a bug
        // }

        // Logger.recordOutput("drive controller left x", () -> driverController.getLeftX());
        // Logger.recordOutput("drive controller left y", () -> driverController.getLeftY());
        // Logger.recordOutput("drive controller right x", () -> driverController.getRightX());

        // driverController.rightStick().debounce(0.5).onTrue(
        //         driveSubsystem.runOnce(
        //                 () -> {
        //                         var latestAlliance = DriverStation.getAlliance();

        //                         boolean side = true;

        //                         if (latestAlliance.isPresent()) {
        //                                 side = latestAlliance.get() != Alliance.Blue;
        //                         }

        //                         driveSubsystem.setDefaultCommand(driveSubsystem.driveFromDriversStation(
        //                                 () -> {
        //                                 return new ChassisSpeeds(
        //                                         deadZone(driverController.getLeftX()) * 1.5,
        //                                         deadZone(driverController.getLeftY()) * 1.5,
        //                                         deadZone(driverController.getRightX())
        //                                                 * Math.PI
        //                                                 * 0.5); // -PI - PI radians per second (-180 - 180
        // degrees/sec)
        //                                 },
        //                                 side));
        //                 }
        //         )
        // );

        boolean side = true; // false for blue, true for red

        double speedMultiplier = 2.5;

        driveSubsystem.setDefaultCommand(driveSubsystem.driveFromDriversStation(
                () -> {
                    return new ChassisSpeeds(
                            deadZone(driverController.getLeftX()) * speedMultiplier,
                            deadZone(driverController.getLeftY()) * speedMultiplier,
                            deadZone(driverController.getRightX())
                                    * Math.PI
                                    * 0.75); // -PI - PI radians per second (-180 - 180 degrees/sec)
                },
                side));

        driverController.rightBumper().whileTrue(climberSubsystem.moveClimberIn());
        driverController.leftBumper().whileTrue(climberSubsystem.moveClimberOut());

        driverController
                .x()
                .debounce(0.1)
                .whileTrue(driveSubsystem.driveAndOrientTowardsReefSide(
                        () -> deadZone(driverController.getLeftX()) * speedMultiplier,
                        () -> deadZone(driverController.getLeftY()) * speedMultiplier,
                        side)); // snaps rotation to the nearest reef side while retaining the ability to move

        driverController
                .y()
                .debounce(0.1)
                .whileTrue(driveSubsystem.driveAndOrientTowardsStationSide(
                        () -> deadZone(driverController.getLeftX()) * speedMultiplier,
                        () -> deadZone(driverController.getLeftY()) * speedMultiplier,
                        side)); // snaps the rotation to the nearest station while retaining the ability to move

        driverController
                .a() // automatically moves to the closest LEFT reef scoring pose
                .debounce(0.1)
                .whileTrue(AutonomousCommands.navigateToNearestLeftBranchScoringPose(driveSubsystem));

        driverController
                .b() // automatically moves to the closest RIGHT reef scoring pose
                .debounce(0.1)
                .whileTrue(AutonomousCommands.navigateToNearestRightBranchScoringPose(driveSubsystem));

        driverController
                .povDown()
                .debounce(.01)
                .whileTrue(driveSubsystem.nudgeBack())
                .onFalse(driveSubsystem.stop());
        driverController
                .povUp()
                .debounce(.01)
                .whileTrue(driveSubsystem.nudgeForward())
                .onFalse(driveSubsystem.stop());
        driverController
                .povLeft()
                .debounce(.01)
                .whileTrue(driveSubsystem.nudgeLeft())
                .onFalse(driveSubsystem.stop());
        driverController
                .povRight()
                .debounce(.01)
                .whileTrue(driveSubsystem.nudgeRight())
                .onFalse(driveSubsystem.stop());
    }

    private void configureOperatorBindings() {

        operatorController.button(10).whileTrue(coralSubsystem.intakeEject().alongWith(algaeSubsystem.intakeEject()));

        operatorController
                .button(1) //  Turns wrist to station pos and intakes coral
                .onTrue(elevatorSubsystem
                        .setStation()
                        .alongWith(wristSubsystem.setStation(), coralSubsystem.setIntaking()));

        operatorController
                .button(2) // Moves to L1 Pos
                .onTrue(elevatorSubsystem
                        .setL1()
                        .alongWith(
                                wristSubsystem.setL1(),
                                coralSubsystem.setEjecting(),
                                algaeSubsystem.setIntaking())); // reversed signes
        operatorController
                .button(12) // Moves to L2 Pos
                .onTrue(elevatorSubsystem
                        .setL2()
                        .alongWith(
                                wristSubsystem.setL2and3(),
                                coralSubsystem.setEjecting(),
                                algaeSubsystem.setEjecting()));
        operatorController
                .button(5) // Moves to L3 Pos
                .onTrue(elevatorSubsystem
                        .setL3()
                        .alongWith(
                                wristSubsystem.setL2and3(),
                                coralSubsystem.setEjecting(),
                                algaeSubsystem.setEjecting()));
        operatorController
                .button(6) // Moves to L4 Pos
                .debounce(0.05)
                .onTrue(elevatorSubsystem
                        .setL4()
                        .alongWith(wristSubsystem.setL4(), coralSubsystem.setEjecting(), algaeSubsystem.setEjecting()));

        // nudges the elevator target a little when pressed
        operatorController.button(4).debounce(0.05).onTrue(elevatorSubsystem.nudgeUp());
        operatorController.button(8).debounce(0.05).onTrue(elevatorSubsystem.nudgeDown());

        operatorController
                .button(11)
                .debounce(0.05)
                .onTrue(elevatorSubsystem
                        .setTargetHeight(ElevatorConstants.ALGAE_LOW_TARGET_HEIGHT)
                        .alongWith(wristSubsystem.setTargetAngle(WristConstants.STOW_POSITION)));

        operatorController
                .button(7) // Moves to Low Algea Pos
                .debounce(0.05)
                .onTrue(elevatorSubsystem
                        .setTargetHeight(ElevatorConstants.ALGAE_HIGH_TARGET_HEIGHT)
                        .alongWith(
                                wristSubsystem.setTargetAngle(WristConstants.STOW_POSITION),
                                algaeSubsystem.setEjecting()));
    }

    private double deadZone(double number) {
        if (Math.abs(number) < 0.05) {
            return 0;
        }
        return number;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        return wristSubsystem
                .zeroWrist()
                .andThen(elevatorSubsystem.zeroEncoders())
                .andThen(AutonomousCommands.scoreAtNearestPlace(
                        driveSubsystem,
                        elevatorSubsystem,
                        wristSubsystem,
                        coralSubsystem,
                        true,
                        ElevatorConstants.LEVEL_3_TARGET_HEIGHT,
                        WristConstants.LEVEL_MID_POSITION));
    }
}
