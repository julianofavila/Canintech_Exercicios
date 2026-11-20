package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SimElevator;
import frc.robot.subsystems.SimHand;
import frc.robot.subsystems.SimIntake;

public class Autos {

    private static SimIntake mSimIntake = new SimIntake(20, 1.5, 3);
    private static SimElevator mSimElevator = new SimElevator(400, 1.5);
    private static SimHand mSimHand = new SimHand(-180, 180, 1.5, 3);

    // private static RobotContainer m_robotContainer;

    public Autos() {
        // m_robotContainer = new RobotContainer();
    }

    public static Command quadrado(TankSubsystem tank) {
        return Commands.sequence(
                tank.driveToPose(2.0, 2.0, 0),
                tank.driveToPose(4.0, 2.0, 90),
                tank.driveToPose(4.0, 5.0, 180),
                tank.driveToPose(2.0, 5.0, -90));
    }

    public static Command reto(TankSubsystem tank) {
        return Commands.sequence(
                tank.driveToPose(2.0, 2.0, 0),
                tank.driveToPose(5.0, 2.0, 0));
    }

    public static Command autoZigZag(TankSubsystem tank) {
        return Commands.sequence(
                tank.driveToPose(2.0, 2.0, 0),
                mSimIntake.CMDintakeOn(),
                tank.driveToPose(4.0, 3.0, 45),
                tank.driveToPose(6.0, 2.0, -45));
    }

    // =========================================================
    // ATIVIDADE 2 - AUTÔNOMO DE 3 ALGAS (2 na NET + 1 no PROCESSOR)
    // =========================================================
    // Todas as posições estão em metros (referencial azul do PathPlanner).
    // Valores marcados com "AJUSTAR" devem ser conferidos no AdvantageScope.

    // Posição inicial (região exigida pelo professor).
    private static final Pose2d START_POSE = new Pose2d(7.13, 1.18, Rotation2d.fromDegrees(180));

    // Algas (mesmas posições da FuelSimulation).
    private static final double ALGA_X = 1.223;
    private static final double ALGA1_Y = 2.200;
    private static final double ALGA2_Y = 4.023;
    private static final double ALGA3_Y = 5.858;

    // Ponto de aproximação (antes de abrir o intake) e ponto de coleta.
    // O intake fica na frente do robô: centro do robô ~0,62 m à frente da alga.
    private static final double APPROACH_X = 2.60;
    private static final double COLLECT_X = ALGA_X + 0.62;

    // Corredores para contornar o REEF (centro ~ 4.49, 4.03) sem colidir.
    private static final double LANE_TOP_Y = 6.40;
    private static final double LANE_BOTTOM_Y = 2.00;

    // NET: lateral esquerda do robô (lado do braço) virada para a barge (+X).
    private static final double NET_X = 7.30; // AJUSTAR
    private static final double NET_Y = 6.00; // AJUSTAR
    private static final double NET_HEADING = -90.0;
    private static final double NET_HAND_ANGLE = 135.0; // AJUSTAR
    private static final double NET_ELEVATOR = 400.0;

    // PROCESSOR: lateral esquerda do robô virada para a parede (-Y).
    private static final double PROC_X = 6.00; // AJUSTAR
    private static final double PROC_Y = 1.00; // AJUSTAR
    private static final double PROC_HEADING = 180.0;
    private static final double PROC_HAND_ANGLE = 90.0; // AJUSTAR
    private static final double PROC_ELEVATOR = 0.0; // AJUSTAR

    // Elevador mínimo para o intake poder coletar sem bater no braço.
    private static final double SAFE_ELEVATOR = 400.0;

    public static Command tresAlgas(TankSubsystem tank, SimIntake intake, SimElevator elevator, SimHand hand) {
        return Commands.sequence(
                // Reset de odometria: o robô "nasce" na posição inicial.
                tank.CMDresetPose(START_POSE),
                prepararColeta(elevator, hand),

                // ---------- Alga 1 -> NET ----------
                tank.driveToPose(APPROACH_X, ALGA1_Y, 180),
                coletar(tank, intake, elevator, COLLECT_X, ALGA1_Y),
                tank.driveToPoint(APPROACH_X, LANE_BOTTOM_Y),
                tank.driveToPoint(6.60, LANE_BOTTOM_Y),
                tank.driveToPose(NET_X, NET_Y, NET_HEADING),
                pontuar(tank, elevator, hand, NET_ELEVATOR, NET_HAND_ANGLE),
                prepararColeta(elevator, hand),

                // ---------- Alga 2 -> NET ----------
                tank.driveToPoint(3.00, LANE_TOP_Y),
                tank.driveToPose(APPROACH_X, ALGA2_Y, 180),
                coletar(tank, intake, elevator, COLLECT_X, ALGA2_Y),
                tank.driveToPoint(3.00, LANE_TOP_Y),
                tank.driveToPose(NET_X, NET_Y, NET_HEADING),
                pontuar(tank, elevator, hand, NET_ELEVATOR, NET_HAND_ANGLE),
                prepararColeta(elevator, hand),

                // ---------- Alga 3 -> PROCESSOR ----------
                tank.driveToPose(APPROACH_X, ALGA3_Y, 180),
                coletar(tank, intake, elevator, COLLECT_X, ALGA3_Y),
                tank.driveToPoint(APPROACH_X, ALGA3_Y),
                tank.driveToPoint(APPROACH_X, LANE_BOTTOM_Y),
                tank.driveToPose(PROC_X, PROC_Y, PROC_HEADING),
                pontuar(tank, elevator, hand, PROC_ELEVATOR, PROC_HAND_ANGLE),
                prepararColeta(elevator, hand));
    }

    /** Sobe o elevador e centraliza o braço (posição segura para o intake). */
    private static Command prepararColeta(SimElevator elevator, SimHand hand) {
        return Commands.sequence(
                elevator.CMDsetPosition(SAFE_ELEVATOR, 3),
                Commands.waitUntil(() -> SimElevator.getRealPosition() > SAFE_ELEVATOR - 10).withTimeout(2),
                hand.CMDsetAngle(0, 3),
                Commands.waitUntil(() -> Math.abs(SimHand.getRealAngle()) < 5).withTimeout(2));
    }

    /**
     * Abre o intake, avança até a alga e espera ela chegar ao braço.
     * A transferência intake -> braço é feita pelo trigger autoHand() do RobotContainer.
     */
    private static Command coletar(TankSubsystem tank, SimIntake intake, SimElevator elevator, double x, double y) {
        return Commands.sequence(
                // Regra de proteção: só coleta com o elevador em cima.
                Commands.waitUntil(() -> SimElevator.getRealPosition() >= 200).withTimeout(2),
                intake.CMDintakeOn(),
                tank.driveToPoint(x, y),
                Commands.waitUntil(tank::isHandCollected).withTimeout(4),
                intake.CMDintakeOff());
    }

    /** Sobe o elevador, gira o braço para o alvo, arremessa e recolhe o braço. */
    private static Command pontuar(TankSubsystem tank, SimElevator elevator, SimHand hand,
            double elevatorPos, double handAngle) {
        return Commands.sequence(
                // Gira o braço só com o elevador em cima, para não bater no robô/intake.
                elevator.CMDsetPosition(SAFE_ELEVATOR, 3),
                Commands.waitUntil(() -> SimElevator.getRealPosition() > SAFE_ELEVATOR - 10).withTimeout(2),
                hand.CMDsetAngle(handAngle, 3),
                Commands.waitUntil(() -> Math.abs(SimHand.getRealAngle() - handAngle) < 5).withTimeout(2),
                elevator.CMDsetPosition(elevatorPos, 3),
                Commands.waitUntil(() -> Math.abs(SimElevator.getRealPosition() - elevatorPos) < 10).withTimeout(2),
                hand.CMDsetVelocity(1),
                tank.depositFuelCommand(),
                Commands.waitSeconds(0.5),
                hand.CMDsetVelocity(0));
    }

    public static Command depositGamePiece(TankSubsystem tank) {
        return Commands.sequence(
                mSimHand.CMDsetVelocity(1),
                tank.depositFuelCommand(),
                Commands.waitSeconds(1),
                mSimHand.CMDsetVelocity(0));
    }

}
