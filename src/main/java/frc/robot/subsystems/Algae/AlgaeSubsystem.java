package frc.robot.subsystems.Algae;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {

    public static class AlgaeConstants {
        public static final double ALGAE_INTAKE_SPEED = 0.5;
        public static final double ALGAE_EJECT_SPEED = -0.5;

        public static final SparkBaseConfig ALGAE_MOTOR_CONFIG =
                new SparkMaxConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kCoast);
    }

    private final SparkMax rightMotor = new SparkMax(13, SparkMax.MotorType.kBrushless); // initilizes lead motor
    private final SparkMax leftMotor = new SparkMax(0, SparkMax.MotorType.kBrushless);

    public AlgaeSubsystem() {
        rightMotor.configure(
                AlgaeConstants.ALGAE_MOTOR_CONFIG,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        leftMotor.configure(
                AlgaeConstants.ALGAE_MOTOR_CONFIG.follow(14, true),
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
    }

    /** starts the motor intaking and then sets it to stop when interupted */
    public Command intake() {
        return startEnd(() -> rightMotor.set(AlgaeConstants.ALGAE_INTAKE_SPEED), () -> rightMotor.set(0));
    }

    /** starts the motor ejecting and then sets it to stop when interupted */
    public Command eject() {
        return startEnd(() -> rightMotor.set(AlgaeConstants.ALGAE_EJECT_SPEED), () -> rightMotor.set(0));
    }

    /** sets the intake motor to start intaking */
    public Command setIntaking() {
        return runOnce(() -> rightMotor.set(AlgaeConstants.ALGAE_INTAKE_SPEED));
    }

    /** sets the intake motor to start ejecting */
    public Command setEjecting() {
        return runOnce(() -> rightMotor.set(AlgaeConstants.ALGAE_EJECT_SPEED));
    }

    /** sets the intake motor to 0 */
    public Command stop() {
        return runOnce(() -> rightMotor.set(0));
    }
}
