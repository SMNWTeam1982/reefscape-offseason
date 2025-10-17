package frc.robot.subsystems.Wrist;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {

    public static class CoralConstants {
        public static final double CORAL_INTAKE_SPEED = 0.6;
        public static final double CORAL_EJECT_SPEED = -0.4;
        public static final SparkBaseConfig INTAKE_MOTOR_CONFIG =
                new SparkMaxConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kBrake);
    }

    private final SparkMax intakeMotor = new SparkMax(16, SparkMax.MotorType.kBrushless);

    private boolean intaking = true;

    public CoralSubsystem() {
        intakeMotor.configure(
                CoralConstants.INTAKE_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** either intakes or ejects based on if setIntaking or setEjecting was ran
     * <p> sets the motor to stop when interupted
     */
    public Command intakeEject() {
        return runEnd(
                () -> {
                    if (intaking) {
                        intakeMotor.set(CoralConstants.CORAL_INTAKE_SPEED);
                    } else {
                        intakeMotor.set(CoralConstants.CORAL_EJECT_SPEED);
                    }
                },
                () -> intakeMotor.set(0));
    }

    /** sets the intake to start intaking when intakeEject is ran */
    public Command setIntaking() {
        return runOnce(() -> {
            intaking = true;
        });
    }

    /** sets the intake to start ejecting when intakeEject is ran */
    public Command setEjecting() {
        return runOnce(() -> {
            intaking = false;
        });
    }

    /** starts the motor intaking and then sets it to stop when interupted */
    public Command intake() {
        return startEnd(() -> intakeMotor.set(CoralConstants.CORAL_INTAKE_SPEED), () -> intakeMotor.set(0));
    }

    /** starts the motor ejecting and then sets it to stop when interupted */
    public Command eject() {
        return startEnd(() -> intakeMotor.set(CoralConstants.CORAL_EJECT_SPEED), () -> intakeMotor.set(0));
    }

    /** sets the intake motor to 0 */
    public Command stop() {
        return runOnce(() -> intakeMotor.set(0));
    }
}
