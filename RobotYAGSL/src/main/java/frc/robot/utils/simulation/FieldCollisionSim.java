package frc.robot.utils.simulation;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Simulador ligero de colisiones 2D del chasis (OBB con bumpers) contra los elementos
 * de la cancha REBUILT 2026 y física 3D (altura Z, Roll, Pitch y frenado) sobre los Bumps.
 * Usa las mismas dimensiones exactas que {@link FuelSim}.
 */
public class FieldCollisionSim {

    public static final double FIELD_LENGTH = FuelSim.FIELD_LENGTH; // 16.51 m
    public static final double FIELD_WIDTH = FuelSim.FIELD_WIDTH;   // 8.04 m

    private static final double HUB_SIDE = 1.20;
    private static final double TRENCH_WIDTH = FuelSim.TRENCH_WIDTH;               // 1.265 m
    private static final double TRENCH_BLOCK_WIDTH = FuelSim.TRENCH_BLOCK_WIDTH;   // 0.305 m
    private static final double TRENCH_CLEARANCE_HEIGHT = FuelSim.TRENCH_HEIGHT;   // 0.565 m
    private static final double TRENCH_BAR_WIDTH = FuelSim.TRENCH_BAR_WIDTH;       // 0.152 m
    private static final double BUMP_PEAK_HEIGHT = 0.165;

    // Coeficientes físicos de respuesta en colisiones y rampas
    private static final double CORNER_TORQUE_GAIN = 0.35; // Giro inducido al golpear con esquina
    private static final double BUMP_SPEED_DAMPING = 0.22; // Reducción de avance al subir/cruzar el Bump
    private static final double SLOPE_GRAVITY_ACCEL = 9.81 * 0.35; // Efecto de pendiente sobre el chasis
    private static final int SOLVER_ITERATIONS = 3;

    /** Caja estática alineada a los ejes del campo (AABB). */
    public record ObstacleBox(double minX, double minY, double maxX, double maxY, String name) {
        public double centerX() {
            return 0.5 * (minX + maxX);
        }

        public double centerY() {
            return 0.5 * (minY + maxY);
        }

        public double halfX() {
            return 0.5 * (maxX - minX);
        }

        public double halfY() {
            return 0.5 * (maxY - minY);
        }
    }

    /** Resultado completo del paso de simulación física del chasis. */
    public record CollisionStepResult(
            Pose2d pose2d,
            Pose3d pose3d,
            boolean isColliding,
            boolean isOnBump,
            String activeContact) {}

    private final double halfLength;
    private final double halfWidth;
    private final double trackHalfLength;
    private final double trackHalfWidth;
    private final List<ObstacleBox> obstacles = new ArrayList<>();

    private boolean collidingLastStep = false;
    private boolean onBumpLastStep = false;
    private String lastContactName = "None";

    public FieldCollisionSim(
            double bumperLengthMeters,
            double bumperWidthMeters,
            double trackLengthMeters,
            double trackWidthMeters,
            double robotHeightMeters) {
        this.halfLength = bumperLengthMeters / 2.0;
        this.halfWidth = bumperWidthMeters / 2.0;
        this.trackHalfLength = trackLengthMeters / 2.0;
        this.trackHalfWidth = trackWidthMeters / 2.0;

        buildRebuiltFieldObstacles(robotHeightMeters > TRENCH_CLEARANCE_HEIGHT);
    }

    private void buildRebuiltFieldObstacles(boolean blockUnderTrenchBars) {
        // 1. Blue Hub y Red Hub (1.20m x 1.20m)
        double blueHubX = 4.61;
        double redHubX = FIELD_LENGTH - 4.61;
        double hubY = FIELD_WIDTH / 2.0;

        obstacles.add(new ObstacleBox(
                blueHubX - HUB_SIDE / 2.0,
                hubY - HUB_SIDE / 2.0,
                blueHubX + HUB_SIDE / 2.0,
                hubY + HUB_SIDE / 2.0,
                "Blue Hub"));

        obstacles.add(new ObstacleBox(
                redHubX - HUB_SIDE / 2.0,
                hubY - HUB_SIDE / 2.0,
                redHubX + HUB_SIDE / 2.0,
                hubY + HUB_SIDE / 2.0,
                "Red Hub"));

        // 2. Los 4 bloques/pilares estructurales de los Trenches
        obstacles.add(new ObstacleBox(
                3.96,
                TRENCH_WIDTH,
                5.18,
                TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
                "Blue Right Trench Pillar"));

        obstacles.add(new ObstacleBox(
                3.96,
                FIELD_WIDTH - 1.57,
                5.18,
                FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH,
                "Blue Left Trench Pillar"));

        obstacles.add(new ObstacleBox(
                FIELD_LENGTH - 5.18,
                TRENCH_WIDTH,
                FIELD_LENGTH - 3.96,
                TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
                "Red Right Trench Pillar"));

        obstacles.add(new ObstacleBox(
                FIELD_LENGTH - 5.18,
                FIELD_WIDTH - 1.57,
                FIELD_LENGTH - 3.96,
                FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH,
                "Red Left Trench Pillar"));

        // 3. Opcional: bloquear paso bajo las barras de los Trenches si el robot excede 0.565m de altura
        if (blockUnderTrenchBars) {
            obstacles.add(new ObstacleBox(
                    4.61 - TRENCH_BAR_WIDTH / 2.0,
                    0.0,
                    4.61 + TRENCH_BAR_WIDTH / 2.0,
                    TRENCH_WIDTH,
                    "Blue Right Trench Bar"));
            obstacles.add(new ObstacleBox(
                    4.61 - TRENCH_BAR_WIDTH / 2.0,
                    FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH,
                    4.61 + TRENCH_BAR_WIDTH / 2.0,
                    FIELD_WIDTH,
                    "Blue Left Trench Bar"));
            obstacles.add(new ObstacleBox(
                    redHubX - TRENCH_BAR_WIDTH / 2.0,
                    0.0,
                    redHubX + TRENCH_BAR_WIDTH / 2.0,
                    TRENCH_WIDTH,
                    "Red Right Trench Bar"));
            obstacles.add(new ObstacleBox(
                    redHubX - TRENCH_BAR_WIDTH / 2.0,
                    FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH,
                    redHubX + TRENCH_BAR_WIDTH / 2.0,
                    FIELD_WIDTH,
                    "Red Left Trench Bar"));
        }
    }

    /**
     * Evalúa rampas (Bumps), colisiones OBB 2D contra bardas y obstáculos, giro por golpe en
     * esquina y construye la {@link Pose3d} resultante del robot para AdvantageScope 3D.
     */
    public CollisionStepResult update(Pose2d preStepPose, Pose2d desiredPostStepPose, double dtSeconds) {
        double dx = desiredPostStepPose.getX() - preStepPose.getX();
        double dy = desiredPostStepPose.getY() - preStepPose.getY();
        double theta = desiredPostStepPose.getRotation().getRadians();

        // 1. Efecto físico al cruzar los Bumps (frenado ligero + pendiente en X)
        double centerFloorZ = getFloorHeightAt(desiredPostStepPose.getX(), desiredPostStepPose.getY());
        double slopeDzDx = getFloorSlopeXAt(desiredPostStepPose.getX(), desiredPostStepPose.getY());
        boolean onBump = centerFloorZ > 1e-4 || Math.abs(slopeDzDx) > 1e-4;

        double x = desiredPostStepPose.getX();
        double y = desiredPostStepPose.getY();

        if (onBump && dtSeconds > 1e-5) {
            x = preStepPose.getX()
                    + dx * (1.0 - BUMP_SPEED_DAMPING)
                    - SLOPE_GRAVITY_ACCEL * slopeDzDx * dtSeconds * dtSeconds;
            y = preStepPose.getY() + dy * (1.0 - BUMP_SPEED_DAMPING * 0.5);
        }

        double vx = dtSeconds > 1e-5 ? (x - preStepPose.getX()) / dtSeconds : 0.0;
        double vy = dtSeconds > 1e-5 ? (y - preStepPose.getY()) / dtSeconds : 0.0;

        boolean collided = false;
        String contactName = "None";

        // 2. Resolver colisiones iterativamente (SAT OBB vs Paredes y Obstáculos AABB)
        for (int iter = 0; iter < SOLVER_ITERATIONS; iter++) {
            double cos = Math.cos(theta);
            double sin = Math.sin(theta);

            // Extensión proyectada del OBB en X e Y del campo
            double extX = halfLength * Math.abs(cos) + halfWidth * Math.abs(sin);
            double extY = halfLength * Math.abs(sin) + halfWidth * Math.abs(cos);

            // 2A. Paredes perimetrales del campo
            if (x - extX < 0.0) {
                double pen = -(x - extX);
                x += pen;
                theta += computeWallCornerSpin(x, y, cos, sin, 1.0, 0.0, vx, vy, dtSeconds);
                vx = Math.max(0.0, vx);
                collided = true;
                contactName = "Blue Alliance Wall";
            } else if (x + extX > FIELD_LENGTH) {
                double pen = (x + extX) - FIELD_LENGTH;
                x -= pen;
                theta += computeWallCornerSpin(x, y, cos, sin, -1.0, 0.0, vx, vy, dtSeconds);
                vx = Math.min(0.0, vx);
                collided = true;
                contactName = "Red Alliance Wall";
            }

            cos = Math.cos(theta);
            sin = Math.sin(theta);
            extY = halfLength * Math.abs(sin) + halfWidth * Math.abs(cos);

            if (y - extY < 0.0) {
                double pen = -(y - extY);
                y += pen;
                theta += computeWallCornerSpin(x, y, cos, sin, 0.0, 1.0, vx, vy, dtSeconds);
                vy = Math.max(0.0, vy);
                collided = true;
                contactName = "Right Side Wall";
            } else if (y + extY > FIELD_WIDTH) {
                double pen = (y + extY) - FIELD_WIDTH;
                y -= pen;
                theta += computeWallCornerSpin(x, y, cos, sin, 0.0, -1.0, vx, vy, dtSeconds);
                vy = Math.min(0.0, vy);
                collided = true;
                contactName = "Left Side Wall";
            }

            // 2B. Obstáculos internos (Hubs y pilares de los Trenches) usando Separating Axis Theorem (SAT)
            for (ObstacleBox box : obstacles) {
                cos = Math.cos(theta);
                sin = Math.sin(theta);

                MtvResult mtv = computeObbVsAabbMtv(x, y, cos, sin, box);
                if (mtv != null) {
                    x += mtv.nx * mtv.overlap;
                    y += mtv.ny * mtv.overlap;

                    // Si el robot se movía hacia el obstáculo, calcular deflexión rotacional de esquina
                    double vInto = -(vx * mtv.nx + vy * mtv.ny);
                    if (vInto > 0.15 && iter == 0) {
                        double rx = mtv.contactX - x;
                        double ry = mtv.contactY - y;
                        double torqueCross = rx * mtv.ny - ry * mtv.nx;
                        theta += CORNER_TORQUE_GAIN * torqueCross * vInto * dtSeconds;

                        // Anular componente normal de velocidad manteniendo deslizamiento tangencial
                        vx += vInto * mtv.nx;
                        vy += vInto * mtv.ny;
                    }

                    collided = true;
                    contactName = box.name();
                }
            }
        }

        theta = MathUtil.angleModulus(theta);
        Pose2d resolvedPose2d = new Pose2d(x, y, new Rotation2d(theta));

        // 3. Construir Pose3d muestreando la altura del piso en las 4 ruedas para Roll y Pitch reales
        Pose3d resolvedPose3d = computeRobotPose3dOnTerrain(resolvedPose2d);
        if (resolvedPose3d.getZ() > 1e-4) {
            onBump = true;
        }

        this.collidingLastStep = collided;
        this.onBumpLastStep = onBump;
        this.lastContactName = contactName;

        SmartDashboard.putBoolean("FieldCollisionSim/IsColliding", collided);
        SmartDashboard.putBoolean("FieldCollisionSim/IsOnBump", onBump);
        SmartDashboard.putString("FieldCollisionSim/ContactElement", contactName);
        SmartDashboard.putNumber("FieldCollisionSim/RobotZHeightM", resolvedPose3d.getZ());
        SmartDashboard.putNumber(
                "FieldCollisionSim/RobotPitchDeg",
                Math.toDegrees(resolvedPose3d.getRotation().getY()));
        SmartDashboard.putNumber(
                "FieldCollisionSim/RobotRollDeg",
                Math.toDegrees(resolvedPose3d.getRotation().getX()));

        return new CollisionStepResult(resolvedPose2d, resolvedPose3d, collided, onBump, contactName);
    }

    private record MtvResult(
            double nx, double ny, double overlap, double contactX, double contactY) {}

    /**
     * Separating Axis Theorem (SAT) 2D entre el OBB del robot y un AABB estático del campo.
     * Devuelve el Minimum Translation Vector (MTV) para expulsar al robot, o {@code null} si no chocan.
     */
    private MtvResult computeObbVsAabbMtv(
            double rx, double ry, double cos, double sin, ObstacleBox box) {
        double dx = rx - box.centerX();
        double dy = ry - box.centerY();

        // Ejes a evaluar: (1,0), (0,1), robotX (cos,sin), robotY (-sin,cos)
        double[][] axes = {
            {1.0, 0.0},
            {0.0, 1.0},
            {cos, sin},
            {-sin, cos}
        };

        double minOverlap = Double.POSITIVE_INFINITY;
        double bestNx = 0.0;
        double bestNy = 0.0;
        int bestAxisIndex = 0;

        for (int i = 0; i < 4; i++) {
            double ax = axes[i][0];
            double ay = axes[i][1];

            double projRobot = halfLength * Math.abs(cos * ax + sin * ay)
                    + halfWidth * Math.abs(-sin * ax + cos * ay);
            double projBox = box.halfX() * Math.abs(ax) + box.halfY() * Math.abs(ay);
            double dist = dx * ax + dy * ay;

            double overlap = (projRobot + projBox) - Math.abs(dist);
            if (overlap <= 0.0) {
                return null; // Eje separador encontrado -> no hay colisión
            }

            if (overlap < minOverlap) {
                minOverlap = overlap;
                double sign = dist >= 0.0 ? 1.0 : -1.0;
                bestNx = ax * sign;
                bestNy = ay * sign;
                bestAxisIndex = i;
            }
        }

        // Estimar punto de contacto para el cálculo de giro por impacto en esquina
        double contactX;
        double contactY;
        if (bestAxisIndex < 2) {
            // Una esquina del robot golpeó una cara del obstáculo
            Translation2d deepestCorner = getDeepestRobotCorner(rx, ry, cos, sin, bestNx, bestNy);
            contactX = deepestCorner.getX();
            contactY = deepestCorner.getY();
        } else {
            // Una esquina del obstáculo golpeó un lado del bumper del robot
            contactX = box.centerX() + (bestNx >= 0.0 ? box.halfX() : -box.halfX());
            contactY = box.centerY() + (bestNy >= 0.0 ? box.halfY() : -box.halfY());
        }

        return new MtvResult(bestNx, bestNy, minOverlap, contactX, contactY);
    }

    private Translation2d getDeepestRobotCorner(
            double rx, double ry, double cos, double sin, double nx, double ny) {
        double bestDot = Double.POSITIVE_INFINITY;
        double bestX = rx;
        double bestY = ry;

        double[] sx = {halfLength, halfLength, -halfLength, -halfLength};
        double[] sy = {halfWidth, -halfWidth, halfWidth, -halfWidth};

        for (int i = 0; i < 4; i++) {
            double cx = rx + sx[i] * cos - sy[i] * sin;
            double cy = ry + sx[i] * sin + sy[i] * cos;
            double dot = (cx - rx) * nx + (cy - ry) * ny;
            if (dot < bestDot) {
                bestDot = dot;
                bestX = cx;
                bestY = cy;
            }
        }
        return new Translation2d(bestX, bestY);
    }

    private double computeWallCornerSpin(
            double rx,
            double ry,
            double cos,
            double sin,
            double nx,
            double ny,
            double vx,
            double vy,
            double dt) {
        double vInto = -(vx * nx + vy * ny);
        if (vInto <= 0.15) {
            return 0.0;
        }
        Translation2d corner = getDeepestRobotCorner(rx, ry, cos, sin, nx, ny);
        double lx = corner.getX() - rx;
        double ly = corner.getY() - ry;
        double cross = lx * ny - ly * nx;
        return CORNER_TORQUE_GAIN * cross * vInto * dt;
    }

    /** Calcula la Pose3d del chasis sobre el terreno (altura Z, Roll y Pitch reales en los Bumps). */
    public Pose3d computeRobotPose3dOnTerrain(Pose2d pose2d) {
        double x = pose2d.getX();
        double y = pose2d.getY();
        double yaw = pose2d.getRotation().getRadians();
        double cos = Math.cos(yaw);
        double sin = Math.sin(yaw);

        // Posición de cada módulo swerve en coordenadas de campo
        double zFL = getFloorHeightAt(
                x + trackHalfLength * cos - trackHalfWidth * sin,
                y + trackHalfLength * sin + trackHalfWidth * cos);
        double zFR = getFloorHeightAt(
                x + trackHalfLength * cos + trackHalfWidth * sin,
                y + trackHalfLength * sin - trackHalfWidth * cos);
        double zBL = getFloorHeightAt(
                x - trackHalfLength * cos - trackHalfWidth * sin,
                y - trackHalfLength * sin + trackHalfWidth * cos);
        double zBR = getFloorHeightAt(
                x - trackHalfLength * cos + trackHalfWidth * sin,
                y - trackHalfLength * sin - trackHalfWidth * cos);
        double zCenterFloor = getFloorHeightAt(x, y);

        double zFront = 0.5 * (zFL + zFR);
        double zBack = 0.5 * (zBL + zBR);
        double zLeft = 0.5 * (zFL + zBL);
        double zRight = 0.5 * (zFR + zBR);

        double wheelAvgZ = 0.25 * (zFL + zFR + zBL + zBR);
        double chassisZ = Math.max(wheelAvgZ, zCenterFloor * 0.85);

        // Convención WPILib NWU: Pitch negativo = frente arriba; Roll positivo = lado izquierdo arriba
        double pitchRad = -Math.atan2(zFront - zBack, 2.0 * trackHalfLength);
        double rollRad = Math.atan2(zLeft - zRight, 2.0 * trackHalfWidth);

        return new Pose3d(
                new Translation3d(x, y, chassisZ),
                new Rotation3d(rollRad, pitchRad, yaw));
    }

    /**
     * Altura Z del piso/Bumps en (x, y), idéntica a las rampas definidas en {@link FuelSim}.
     */
    public static double getFloorHeightAt(double x, double y) {
        if (!isInBumpYRange(y)) {
            return 0.0;
        }

        // Bump Azul: sube de 3.96 -> 4.61 (0.165m) y baja de 4.61 -> 5.18 (0.0m)
        if (x >= 3.96 && x <= 4.61) {
            return BUMP_PEAK_HEIGHT * (x - 3.96) / (4.61 - 3.96);
        } else if (x > 4.61 && x <= 5.18) {
            return BUMP_PEAK_HEIGHT * (5.18 - x) / (5.18 - 4.61);
        }

        // Bump Rojo: sube de (L - 5.18) -> (L - 4.61) y baja de (L - 4.61) -> (L - 3.96)
        double redStart = FIELD_LENGTH - 5.18; // 11.33
        double redPeak = FIELD_LENGTH - 4.61;  // 11.90
        double redEnd = FIELD_LENGTH - 3.96;   // 12.55

        if (x >= redStart && x <= redPeak) {
            return BUMP_PEAK_HEIGHT * (x - redStart) / (redPeak - redStart);
        } else if (x > redPeak && x <= redEnd) {
            return BUMP_PEAK_HEIGHT * (redEnd - x) / (redEnd - redPeak);
        }

        return 0.0;
    }

    private static double getFloorSlopeXAt(double x, double y) {
        if (!isInBumpYRange(y)) {
            return 0.0;
        }
        if (x >= 3.96 && x <= 4.61) {
            return BUMP_PEAK_HEIGHT / (4.61 - 3.96);
        } else if (x > 4.61 && x <= 5.18) {
            return -BUMP_PEAK_HEIGHT / (5.18 - 4.61);
        }

        double redStart = FIELD_LENGTH - 5.18;
        double redPeak = FIELD_LENGTH - 4.61;
        double redEnd = FIELD_LENGTH - 3.96;
        if (x >= redStart && x <= redPeak) {
            return BUMP_PEAK_HEIGHT / (redPeak - redStart);
        } else if (x > redPeak && x <= redEnd) {
            return -BUMP_PEAK_HEIGHT / (redEnd - redPeak);
        }
        return 0.0;
    }

    private static boolean isInBumpYRange(double y) {
        boolean inRightLane = y >= 1.57 && y <= (FIELD_WIDTH / 2.0 - 0.60);
        boolean inLeftLane = y >= (FIELD_WIDTH / 2.0 + 0.60) && y <= (FIELD_WIDTH - 1.57);
        return inRightLane || inLeftLane;
    }

    public boolean isColliding() {
        return collidingLastStep;
    }

    public boolean isOnBump() {
        return onBumpLastStep;
    }

    public String getLastContactName() {
        return lastContactName;
    }
}
