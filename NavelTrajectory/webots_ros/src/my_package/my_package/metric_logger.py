"""Evaluations-Logger fuer Nav2-Navigationsexperimente.

Erfasst Pfadqualitaets- und Ausfuehrungsmetriken fuer den systematischen
Vergleich von Global Planner, Path Smoother und Local Controller.

Abonniert:
    /odom                                  -- Odometrie (Ausfuehrungsmetriken)
    /plan                                  -- Geplanter Pfad (Replanning-Zaehler)
    /plan_smoothed                         -- Geglaetteter Pfad (Pfadqualitaet)
    /amcl_pose                             -- Lokalisierte Position (CTE)
    /navigate_to_pose/_action/status       -- Goal-Status (Start/Ende)

Parameter:
    config_label  (str)   -- Bezeichner der Konfiguration, z.B. 'dwb_baseline'
    output_path   (str)   -- CSV-Ausgabepfad, Default: ~/eval_results.csv

Metriken pro Goal:
    -- Pfadqualitaet (Planner + Smoother) --
    planned_path_length_m        Laenge des geplanten Pfads
    planned_curvature_avg        Mittlere Kruemmung (Menger)
    planned_curvature_max        Maximale Kruemmung (Menger)
    planned_heading_changes_rad  Summe absoluter Richtungswechsel
    planned_path_energy          Integral der quadrierten Kruemmung
    planned_curvature_rate_avg   Mittlere Kruemmungsaenderungsrate
    planned_curvature_rate_max   Maximale Kruemmungsaenderungsrate

    -- Ausfuehrungsqualitaet (Controller) --
    execution_time_s             Gesamtzeit vom Goal-Start bis Ergebnis
    path_length_executed_m       Tatsaechlich gefahrene Strecke (Odometrie)
    path_length_ratio            Verhaeltnis ausgefuehrt/geplant (>1 = Umwege)
    avg_linear_velocity_ms       Mittlere Vorwaertsgeschwindigkeit
    avg_cross_track_error_m      Mittlere Abweichung vom geplanten Pfad
    max_cross_track_error_m      Maximale Abweichung vom geplanten Pfad
    replanning_events            Anzahl Replannings waehrend Ausfuehrung
    recovery_events              Anzahl Recovery-Behaviors (Spin, Backup, Wait)
    goal_status                  SUCCEEDED oder ABORTED
"""

import csv
import math
import os
import time
from typing import List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class MetricLogger(Node):
    """Erfasst und protokolliert Navigationsmetriken pro Goal."""

    def __init__(self):
        super().__init__('metric_logger')

        # -- Parameter --
        self.declare_parameter('config_label', 'unnamed')
        self.declare_parameter(
            'output_path',
            os.path.expanduser('~/eval_results.csv'),
        )
        self._config_label = (
            self.get_parameter('config_label').get_parameter_value().string_value
        )
        self._csv_path = (
            self.get_parameter('output_path').get_parameter_value().string_value
        )

        # -- Zustand --
        self._start_time = None
        self._path_length_executed = 0.0
        self._last_pose = None
        self._linear_velocities: List[float] = []
        self._cross_track_errors: List[float] = []
        self._replanning_count = 0
        self._recovery_count = 0
        self._current_plan = None
        # Referenz-Plan fuer CTE: wird einmalig beim ersten Plan eingefroren
        # und nicht durch Replannings ueberschrieben.
        self._cte_reference_plan = None
        self._planned_path_length = None
        self._planned_curvature_avg = None
        self._planned_curvature_max = None
        # Smoother-spezifische Metriken
        self._planned_heading_changes = None
        self._planned_path_energy = None
        self._planned_curvature_rate_avg = None
        self._planned_curvature_rate_max = None
        self._last_status = None
        self._goal_active = False
        # Roboterposition im map-Frame (von /amcl_pose).
        self._map_pos = None

        # -- Subscriptions --
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        # /plan nur noch fuer Replanning-Erkennung
        self.create_subscription(Path, '/plan', self._plan_cb, 10)
        # /plan_smoothed fuer Pfadqualitaets-Metriken
        self.create_subscription(Path, '/plan_smoothed', self._plan_smoothed_cb, 10)
        # amcl_pose liefert die lokalisierte Position im map-Frame
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_cb,
            10,
        )
        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self._status_cb,
            10,
        )

        self.get_logger().info(
            f'MetricLogger bereit. Config: {self._config_label}, '
            f'Output: {self._csv_path}'
        )

    # -------------------------------------------------------------------
    # Topic-Callbacks
    # -------------------------------------------------------------------

    def _plan_cb(self, msg: Path):
        """Zaehlt Replanning-Events.

        Wird nur noch genutzt, um Replanning-Events zu zaehlen.
        Die Pfadqualitaet wird von _plan_smoothed_cb erfasst.
        """
        if not msg.poses:
            return

        if self._goal_active:
            if self._current_plan is not None:
                self._replanning_count += 1
                self.get_logger().debug(
                    f'Replanning-Event #{self._replanning_count} erkannt.'
                )

    def _plan_smoothed_cb(self, msg: Path):
        """Verarbeitet den geglaetteten Pfad und berechnet Qualitaetsmetriken.

        Beim ersten geglaetteten Plan pro Goal werden alle Pfadqualitaets-
        metriken berechnet und der Pfad als CTE-Referenz eingefroren.

        Args:
            msg: Neue Path-Nachricht vom Path Smoother.
        """
        if not msg.poses:
            return

        self._current_plan = msg

        # Pfadqualitaet + CTE-Referenz nur vom ERSTEN Plan speichern
        if self._planned_path_length is None and self._goal_active:
            # Basismetriken
            length = self._compute_path_length(msg)
            curvature_avg, curvature_max = self._compute_path_curvature(msg)
            self._planned_path_length = length
            self._planned_curvature_avg = curvature_avg
            self._planned_curvature_max = curvature_max

            # Smoother-spezifische Metriken
            self._planned_heading_changes = self._compute_heading_changes(msg)
            self._planned_path_energy = self._compute_path_energy(msg)
            cr_avg, cr_max = self._compute_curvature_rate(msg)
            self._planned_curvature_rate_avg = cr_avg
            self._planned_curvature_rate_max = cr_max

            # CTE wird immer gegen den initialen (geglaetteten) Plan gemessen.
            self._cte_reference_plan = msg
            self.get_logger().debug(
                f'Erster geglaetteter Plan empfangen: {length:.3f} m, '
                f'{len(msg.poses)} Poses -- Metriken & CTE-Referenz eingefroren.'
            )

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        """Speichert die aktuell lokalisierte Position im map-Frame.

        Args:
            msg: Aktuelle AMCL-Pose im map-Frame.
        """
        self._map_pos = msg.pose.pose.position

    def _odom_cb(self, msg: Odometry):
        """Akkumuliert Ausfuehrungsmetriken aus Odometrie.

        Pfadlaenge wird aus /odom akkumuliert (hochfrequent, glatt).
        CTE wird gegen /amcl_pose im map-Frame berechnet.

        Args:
            msg: Aktuelle Odometrie-Nachricht.
        """
        if not self._goal_active:
            return

        pos = msg.pose.pose.position
        vx = msg.twist.twist.linear.x

        # Geschwindigkeit aufzeichnen
        self._linear_velocities.append(abs(vx))

        # Pfadlaenge aus odom akkumulieren
        if self._last_pose is not None:
            dx = pos.x - self._last_pose.x
            dy = pos.y - self._last_pose.y
            self._path_length_executed += math.hypot(dx, dy)
        self._last_pose = pos

        # Cross-Track-Error: nur waehrend echter Vorwaertsbewegung und nur
        # wenn eine map-Frame-Position verfuegbar ist.
        _CTE_VEL_THRESHOLD = 0.05  # m/s
        if (
            abs(vx) >= _CTE_VEL_THRESHOLD
            and self._map_pos is not None
            and self._cte_reference_plan
            and self._cte_reference_plan.poses
        ):
            cte = self._min_dist_to_path(self._map_pos, self._cte_reference_plan)
            self._cross_track_errors.append(cte)
            # Debug: jeden 50. Sample loggen
            if len(self._cross_track_errors) % 50 == 0:
                self.get_logger().debug(
                    f'CTE-Sample #{len(self._cross_track_errors)}: '
                    f'{cte:.3f} m  |  vx={vx:.3f} m/s  |  '
                    f'map_pos=({self._map_pos.x:.2f}, {self._map_pos.y:.2f})'
                )

    def _status_cb(self, msg: GoalStatusArray):
        """Erkennt Goal-Zustandsuebergaenge und loest CSV-Logging aus.

        Args:
            msg: Aktueller GoalStatusArray des NavigateToPose-Actions.
        """
        if not msg.status_list:
            return

        status = msg.status_list[-1].status
        if status == self._last_status:
            return
        self._last_status = status

        if status == GoalStatus.STATUS_EXECUTING:
            if not self._goal_active:
                self.get_logger().info('Goal EXECUTING -- starte Messung.')
                self._reset()
                self._goal_active = True
                self._start_time = self.get_clock().now()

        elif status == GoalStatus.STATUS_SUCCEEDED:
            if not self._goal_active:
                self.get_logger().warn(
                    'SUCCEEDED ohne vorheriges EXECUTING erkannt. '
                    'Ausfuehrungsmetriken unvollstaendig.'
                )
                self._goal_active = True
                if self._start_time is None:
                    self._start_time = self.get_clock().now()
            self.get_logger().info('Goal SUCCEEDED -- logge Metriken.')
            self._log_and_reset('SUCCEEDED')

        elif status == GoalStatus.STATUS_ABORTED:
            self._recovery_count += 1
            self.get_logger().info(
                f'Goal ABORTED (Recovery #{self._recovery_count})'
            )

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal CANCELED -- logge Metriken.')
            self._log_and_reset('CANCELED')

    # -------------------------------------------------------------------
    # Logging & Reset
    # -------------------------------------------------------------------

    def _log_and_reset(self, goal_status: str):
        """Berechnet Metriken, schreibt CSV und setzt den Zustand zurueck.

        Args:
            goal_status: Endstatus des Goals ('SUCCEEDED', 'CANCELED').
        """
        if not self._goal_active:
            return

        results = self._compute_results(goal_status)
        self.get_logger().info(f'=== EVAL === {results}')
        self._write_csv(results)
        self._reset()

    def _compute_results(self, goal_status: str) -> dict:
        """Berechnet alle Metriken aus den akkumulierten Daten.

        Args:
            goal_status: Endstatus des Goals.

        Returns:
            Dictionary mit gerundeten Metrik-Werten.
        """
        elapsed = 0.0
        if self._start_time is not None:
            dt = self.get_clock().now() - self._start_time
            elapsed = dt.nanoseconds / 1e9

        avg_cte = (
            sum(self._cross_track_errors) / len(self._cross_track_errors)
            if self._cross_track_errors else 0.0
        )
        max_cte = (
            max(self._cross_track_errors)
            if self._cross_track_errors else 0.0
        )
        avg_vel = (
            sum(self._linear_velocities) / len(self._linear_velocities)
            if self._linear_velocities else 0.0
        )

        # Pfadlaengenverhaeltnis: ausgefuehrt / geplant
        path_ratio = 0.0
        if self._planned_path_length and self._planned_path_length > 0.01:
            path_ratio = self._path_length_executed / self._planned_path_length

        return {
            'config_label':              self._config_label,
            'goal_status':               goal_status,
            # -- Pfadqualitaet (Planner + Smoother) --
            'planned_path_length_m':     round(self._planned_path_length or 0.0, 3),
            'planned_curvature_avg':     round(self._planned_curvature_avg or 0.0, 4),
            'planned_curvature_max':     round(self._planned_curvature_max or 0.0, 4),
            'planned_heading_changes_rad': round(self._planned_heading_changes or 0.0, 4),
            'planned_path_energy':       round(self._planned_path_energy or 0.0, 6),
            'planned_curvature_rate_avg': round(self._planned_curvature_rate_avg or 0.0, 4),
            'planned_curvature_rate_max': round(self._planned_curvature_rate_max or 0.0, 4),
            # -- Ausfuehrungsqualitaet (Controller) --
            'execution_time_s':          round(elapsed, 2),
            'path_length_executed_m':    round(self._path_length_executed, 3),
            'path_length_ratio':         round(path_ratio, 3),
            'avg_linear_velocity_ms':    round(avg_vel, 4),
            'avg_cross_track_error_m':   round(avg_cte, 4),
            'max_cross_track_error_m':   round(max_cte, 4),
            'replanning_events':         self._replanning_count,
            'recovery_events':           self._recovery_count,
        }

    def _write_csv(self, results: dict):
        """Schreibt eine Ergebniszeile in die CSV-Datei.

        Fuegt einen Header ein, wenn die Datei leer oder neu ist.

        Args:
            results: Dictionary mit Metrik-Werten.
        """
        file_exists = os.path.isfile(self._csv_path)
        write_header = not file_exists or os.path.getsize(self._csv_path) == 0

        with open(self._csv_path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=results.keys())
            if write_header:
                writer.writeheader()
            writer.writerow(results)

        self.get_logger().info(f'CSV geschrieben: {self._csv_path}')

    def _reset(self):
        """Setzt alle Akkumulatoren fuer das naechste Goal zurueck."""
        self._start_time = None
        self._path_length_executed = 0.0
        self._last_pose = None
        self._linear_velocities = []
        self._cross_track_errors = []
        self._replanning_count = 0
        self._recovery_count = 0
        self._current_plan = None
        self._cte_reference_plan = None
        self._planned_path_length = None
        self._planned_curvature_avg = None
        self._planned_curvature_max = None
        self._planned_heading_changes = None
        self._planned_path_energy = None
        self._planned_curvature_rate_avg = None
        self._planned_curvature_rate_max = None
        self._goal_active = False
        # map_pos NICHT zuruecksetzen -- AMCL laeuft kontinuierlich

    # -------------------------------------------------------------------
    # Pfadanalyse-Methoden
    # -------------------------------------------------------------------

    @staticmethod
    def _compute_path_length(plan: Path) -> float:
        """Berechnet die Laenge eines geplanten Pfads.

        Args:
            plan: Geplanter Pfad.

        Returns:
            Pfadlaenge in Metern.
        """
        length = 0.0
        poses = plan.poses
        for i in range(1, len(poses)):
            p0 = poses[i - 1].pose.position
            p1 = poses[i].pose.position
            length += math.hypot(p1.x - p0.x, p1.y - p0.y)
        return length

    @staticmethod
    def _compute_path_curvature(plan: Path) -> Tuple[float, float]:
        """Berechnet mittlere und maximale Kruemmung eines Pfads.

        Kruemmung wird ueber drei aufeinanderfolgende Punkte als
        Kehrwert des Umkreisradius approximiert (Menger-Kruemmung).

        Args:
            plan: Geplanter Pfad.

        Returns:
            Tuple (mittlere Kruemmung, maximale Kruemmung).
            Gibt (0.0, 0.0) zurueck, wenn der Pfad zu kurz ist.
        """
        poses = plan.poses
        if len(poses) < 3:
            return 0.0, 0.0

        curvatures = []
        # Schrittweite > 1, um Rauschen bei sehr dichten Pfaden zu reduzieren
        step = max(1, len(poses) // 200)

        for i in range(step, len(poses) - step, step):
            p0 = poses[i - step].pose.position
            p1 = poses[i].pose.position
            p2 = poses[i + step].pose.position

            # Flaecheninhalt des Dreiecks (2 * A)
            area2 = abs(
                (p1.x - p0.x) * (p2.y - p0.y)
                - (p2.x - p0.x) * (p1.y - p0.y)
            )

            # Seitenlaengen
            a = math.hypot(p1.x - p0.x, p1.y - p0.y)
            b = math.hypot(p2.x - p1.x, p2.y - p1.y)
            c = math.hypot(p2.x - p0.x, p2.y - p0.y)

            denom = a * b * c
            if denom < 1e-9:
                continue

            # Menger-Kruemmung: kappa = 2*A / (a*b*c)
            kappa = area2 / denom
            curvatures.append(kappa)

        if not curvatures:
            return 0.0, 0.0

        return (
            sum(curvatures) / len(curvatures),
            max(curvatures),
        )

    @staticmethod
    def _compute_heading_changes(plan: Path) -> float:
        """Berechnet die Summe absoluter Richtungswechsel entlang des Pfads.

        Misst die kumulative Orientierungsaenderung: ein gerader Pfad
        ergibt ~0, ein Gitterpfad mit 90-Grad-Ecken ergibt hohe Werte.
        Ein gut geglaetteter Pfad liegt dazwischen mit sanften Uebergaengen.

        Einheit: Radiant. Niedrigere Werte = glatterer Pfad.

        Args:
            plan: Geplanter Pfad.

        Returns:
            Summe der absoluten Heading-Aenderungen in Radiant.
        """
        poses = plan.poses
        if len(poses) < 3:
            return 0.0

        total_change = 0.0
        prev_heading = math.atan2(
            poses[1].pose.position.y - poses[0].pose.position.y,
            poses[1].pose.position.x - poses[0].pose.position.x,
        )

        for i in range(2, len(poses)):
            dx = poses[i].pose.position.x - poses[i - 1].pose.position.x
            dy = poses[i].pose.position.y - poses[i - 1].pose.position.y

            # Segmente mit Nulllaenge ueberspringen (duplizierte Punkte)
            if abs(dx) < 1e-9 and abs(dy) < 1e-9:
                continue

            heading = math.atan2(dy, dx)

            # Winkeldifferenz auf [-pi, pi] normalisieren
            delta = heading - prev_heading
            delta = (delta + math.pi) % (2.0 * math.pi) - math.pi

            total_change += abs(delta)
            prev_heading = heading

        return total_change

    @staticmethod
    def _compute_path_energy(plan: Path) -> float:
        """Berechnet das Kruemmungsintegral (Path Energy) des Pfads.

        Path Energy = Summe(kappa_i^2 * ds_i), wobei ds_i die Segmentlaenge
        und kappa_i die lokale Menger-Kruemmung ist.

        Dies ist das Standard-Optimierungskriterium fuer Spline-Smoother:
        niedrigere Werte bedeuten einen energetisch guenstigeren, glatteren Pfad.

        Args:
            plan: Geplanter Pfad.

        Returns:
            Path Energy (dimensionslos). 0.0 fuer zu kurze Pfade.
        """
        poses = plan.poses
        if len(poses) < 3:
            return 0.0

        energy = 0.0
        for i in range(1, len(poses) - 1):
            p0 = poses[i - 1].pose.position
            p1 = poses[i].pose.position
            p2 = poses[i + 1].pose.position

            # Menger-Kruemmung an Punkt i
            area2 = abs(
                (p1.x - p0.x) * (p2.y - p0.y)
                - (p2.x - p0.x) * (p1.y - p0.y)
            )
            a = math.hypot(p1.x - p0.x, p1.y - p0.y)
            b = math.hypot(p2.x - p1.x, p2.y - p1.y)
            c = math.hypot(p2.x - p0.x, p2.y - p0.y)

            denom = a * b * c
            if denom < 1e-9:
                continue

            kappa = area2 / denom

            # ds = mittlere Segmentlaenge um Punkt i
            ds = 0.5 * (a + b)
            energy += kappa * kappa * ds

        return energy

    @staticmethod
    def _compute_curvature_rate(plan: Path) -> Tuple[float, float]:
        """Berechnet die Kruemmungsaenderungsrate entlang des Pfads.

        Misst, wie abrupt sich die Kruemmung aendert -- analog zum
        kinematischen "Ruck" (jerk). Hohe Werte deuten auf scharfe
        Uebergaenge hin, die ein Differential-Drive-Roboter nur durch
        Abbremsen bewaltigen kann.

        Args:
            plan: Geplanter Pfad.

        Returns:
            Tuple (mittlere Rate, maximale Rate) in rad/m^2.
            Gibt (0.0, 0.0) zurueck, wenn der Pfad zu kurz ist.
        """
        poses = plan.poses
        if len(poses) < 4:
            return 0.0, 0.0

        # Schritt 1: Kruemmung an jedem inneren Punkt berechnen
        curvatures = []
        arc_lengths = []

        for i in range(1, len(poses) - 1):
            p0 = poses[i - 1].pose.position
            p1 = poses[i].pose.position
            p2 = poses[i + 1].pose.position

            area2 = abs(
                (p1.x - p0.x) * (p2.y - p0.y)
                - (p2.x - p0.x) * (p1.y - p0.y)
            )
            a = math.hypot(p1.x - p0.x, p1.y - p0.y)
            b = math.hypot(p2.x - p1.x, p2.y - p1.y)
            c = math.hypot(p2.x - p0.x, p2.y - p0.y)

            denom = a * b * c
            if denom < 1e-9:
                curvatures.append(0.0)
                arc_lengths.append(0.5 * (a + b))
                continue

            kappa = area2 / denom
            curvatures.append(kappa)
            arc_lengths.append(0.5 * (a + b))

        # Schritt 2: dkappa/ds zwischen aufeinanderfolgenden Punkten
        rates = []
        for i in range(1, len(curvatures)):
            ds = 0.5 * (arc_lengths[i - 1] + arc_lengths[i])
            if ds < 1e-9:
                continue
            rate = abs(curvatures[i] - curvatures[i - 1]) / ds
            rates.append(rate)

        if not rates:
            return 0.0, 0.0

        return (
            sum(rates) / len(rates),
            max(rates),
        )

    @staticmethod
    def _min_dist_to_path(pos, plan: Path) -> float:
        """Berechnet den minimalen Abstand einer Position zum Pfad.

        Nutzt Punkt-zu-Segment-Distanz fuer hoehere Genauigkeit
        als reine Punkt-zu-Punkt-Distanz.

        Args:
            pos:  Aktuelle Position (geometry_msgs/Point).
            plan: Aktueller geplanter Pfad.

        Returns:
            Minimaler euklidischer Abstand in Metern.
        """
        min_d = float('inf')
        poses = plan.poses

        for i in range(1, len(poses)):
            ax = poses[i - 1].pose.position.x
            ay = poses[i - 1].pose.position.y
            bx = poses[i].pose.position.x
            by = poses[i].pose.position.y

            # Projektion von pos auf Segment [a, b]
            abx = bx - ax
            aby = by - ay
            apx = pos.x - ax
            apy = pos.y - ay

            ab_sq = abx * abx + aby * aby
            if ab_sq < 1e-12:
                d = math.hypot(apx, apy)
            else:
                t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab_sq))
                proj_x = ax + t * abx
                proj_y = ay + t * aby
                d = math.hypot(pos.x - proj_x, pos.y - proj_y)

            if d < min_d:
                min_d = d

        return min_d


def main(args=None):
    rclpy.init(args=args)
    node = MetricLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()