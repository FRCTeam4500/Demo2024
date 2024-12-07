package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.SwerveModule.SwerveModuleConfig;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.CANConstants.*;
import static com.ctre.phoenix6.signals.InvertedValue.*;
import static com.revrobotics.spark.SparkLowLevel.MotorType.*;
import static com.revrobotics.spark.SparkBase.ResetMode.*;
import static com.revrobotics.spark.SparkBase.PersistMode.*;

public class SwerveConstants {
    /** Drive rotations per motor rotation */
    public static final double DRIVE_RATIO = 1/7.5;
    /** Angle rotations per motor rotation */
    public static final double ANGLE_RATIO = 1/6.75;

    public static final double MAX_LINEAR_SPEED_MPS = 6;
    public static final double WHEEL_DIAMETER_METERS = 0.1016;

    public static final ChassisSpeeds MAX_SPEEDS = new ChassisSpeeds(3, 3, Math.PI / 32);
    
    public static final TalonFXConfiguration driveConfig =
        new TalonFXConfiguration()
            .withSlot1(new Slot1Configs()
                .withKP(0.11)
                .withKI(0.5)
                .withKD(0.0001)
                .withKV(0.12))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(35)
                .withSupplyCurrentLimitEnable(true)
            );


    public static final SwerveMotor FRONT_LEFT_DRIVE_MOTOR =
        SwerveMotor.fromTalonFX(
            new TalonFX(SWERVE_FRONT_LEFT_DRIVE_ID),
            driveConfig.withMotorOutput(new MotorOutputConfigs()
                .withInverted(Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
        );
    public static final SwerveMotor FRONT_LEFT_ANGLE_MOTOR =
        SwerveMotor.fromSparkMax(
            new SparkMax(SWERVE_FRONT_LEFT_ANGLE_ID, kBrushless),
            motor -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.closedLoop.p(0.7);
                motor.configure(config, kResetSafeParameters, kPersistParameters);
            }
        );
    public static final SwerveModuleConfig FRONT_LEFT_MODULE_CONFIG = new SwerveModuleConfig(
        new Translation2d(0.2974, 0.2974), 0.1016, 1/7.5, 1/6.75
    );

    public static final SwerveMotor FRONT_RIGHT_DRIVE_MOTOR =
        SwerveMotor.fromTalonFX(
            new TalonFX(SWERVE_FRONT_RIGHT_DRIVE_ID),
            driveConfig.withMotorOutput(new MotorOutputConfigs()
                .withInverted(CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
        );
    public static final SwerveMotor FRONT_RIGHT_ANGLE_MOTOR =
        SwerveMotor.fromSparkMax(
            new SparkMax(SWERVE_FRONT_RIGHT_ANGLE_ID, kBrushless),
            motor -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.closedLoop.p(0.8);
                motor.configure(config, kResetSafeParameters, kPersistParameters);
            }
        );
    public static final SwerveModuleConfig FRONT_RIGHT_MODULE_CONFIG = new SwerveModuleConfig(
        new Translation2d(0.2974, -0.2974), 0.1016, 1/7.5, 1/6.75
    );

    public static final SwerveMotor BACK_LEFT_DRIVE_MOTOR =
        SwerveMotor.fromTalonFX(
            new TalonFX(SWERVE_BACK_LEFT_DRIVE_ID),
            driveConfig.withMotorOutput(new MotorOutputConfigs()
                .withInverted(Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
        );
    public static final SwerveMotor BACK_LEFT_ANGLE_MOTOR =
        SwerveMotor.fromSparkMax(
            new SparkMax(SWERVE_BACK_LEFT_ANGLE_ID, kBrushless),
            motor -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.closedLoop.p(0.75);
                motor.configure(config, kResetSafeParameters, kPersistParameters);
            }
        );
    public static final SwerveModuleConfig BACK_LEFT_MODULE_CONFIG = new SwerveModuleConfig(
        new Translation2d(-0.2974, 0.2974), 0.1016, 1/7.5, 1/6.75
    );

    public static final SwerveMotor BACK_RIGHT_DRIVE_MOTOR =
        SwerveMotor.fromTalonFX(
            new TalonFX(SWERVE_BACK_RIGHT_DRIVE_ID),
            driveConfig.withMotorOutput(new MotorOutputConfigs()
                .withInverted(CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
        );
    public static final SwerveMotor BACK_RIGHT_ANGLE_MOTOR =
        SwerveMotor.fromSparkMax(
            new SparkMax(SWERVE_BACK_RIGHT_ANGLE_ID, kBrushless),
            motor -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.closedLoop.p(0.8);
                motor.configure(config, kResetSafeParameters, kPersistParameters);
            }
        );
    public static final SwerveModuleConfig BACK_RIGHT_MODULE_CONFIG = new SwerveModuleConfig(
        new Translation2d(-0.2974, -0.2974), 0.1016, 1/7.5, 1/6.75
    );
}