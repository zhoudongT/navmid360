# Copyright (c) 2026, extracted and adapted for dong_3d_nav.
# Original DWA-style logic from MoveBase3D (local_planner_3d/local_planner_3d/nodes/local_planner_node.py)
# https://github.com/WilsonGuo/MoveBase3D — BSD-style upstream license applies to derived portions.

"""Pure-numpy DWA trajectory sampling (no ROS imports)."""

from __future__ import annotations

import math
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from functools import partial
from typing import Iterable, Sequence

import numpy as np


@dataclass
class DWAConfig:
    predict_time: float = 2.0
    dt: float = 0.1
    safe_distance: float = 0.3
    goal_weight: float = 1.0
    speed_weight: float = 0.5
    obstacle_weight: float = 1.9
    v_min: float = -0.2
    v_max: float = 0.6
    omega_min: float = -1.0
    omega_max: float = 1.0
    v_sample_step: float = 0.1
    omega_sample_step: float = 0.1
    v_cap: float = 0.6
    omega_cap: float = 1.0
    thread_workers: int = 8


def simulate_trajectory(
    x: float,
    y: float,
    theta: float,
    v: float,
    omega: float,
    predict_time: float,
    dt: float,
) -> np.ndarray:
    """Unicycle roll-out in map frame (same structure as MoveBase3D)."""
    time_steps = np.arange(0.0, predict_time, dt)
    n_steps = len(time_steps)
    trajectory = np.zeros((n_steps, 3), dtype=np.float64)

    if abs(omega) < 1e-9:
        trajectory[:, 0] = x + v * np.cos(theta) * time_steps
        trajectory[:, 1] = y + v * np.sin(theta) * time_steps
        trajectory[:, 2] = theta
        return trajectory

    dtheta = omega * time_steps
    trajectory[:, 2] = theta + dtheta
    radius = v / omega
    trajectory[:, 0] = x + radius * (np.sin(theta + dtheta) - np.sin(theta))
    trajectory[:, 1] = y - radius * (np.cos(theta + dtheta) - np.cos(theta))
    return trajectory


def evaluate_goal_cost(trajectory: np.ndarray, goal_x: float, goal_y: float) -> float:
    last = trajectory[-1]
    dist = math.hypot(float(last[0]) - goal_x, float(last[1]) - goal_y)
    return -dist


def evaluate_obstacle_cost(
    trajectory: np.ndarray,
    obstacles: np.ndarray,
    safe_distance: float,
    epsilon: float = 1e-6,
    batch_size: int = 50,
) -> float:
    """Vectorized obstacle penalty (ported from MoveBase3D)."""
    if obstacles is None or obstacles.size == 0:
        return 0.0
    traj_xy = np.asarray(trajectory, dtype=np.float64)[:, :2]
    obs = np.asarray(obstacles, dtype=np.float64)
    if obs.ndim == 1:
        if obs.size % 2 != 0:
            return 0.0
        obs = obs.reshape(-1, 2)
    if obs.shape[1] != 2:
        return 0.0

    cost = 0.0
    for i in range(0, len(traj_xy), batch_size):
        batch = traj_xy[i : i + batch_size]
        traj_batch = batch[:, np.newaxis, :]
        obs_b = obs[np.newaxis, :, :]
        dists = np.sqrt(np.sum((traj_batch - obs_b) ** 2, axis=2))
        mask = dists < safe_distance
        if np.any(mask):
            close_dists = dists[mask]
            weights = 1.0 / (close_dists**2 + epsilon)
            cost += float(np.sum(weights * (safe_distance - close_dists) ** 2))
    return cost


def evaluate_one_trajectory(
    v: float,
    omega: float,
    x: float,
    y: float,
    theta: float,
    predict_time: float,
    dt: float,
    goal_x: float,
    goal_y: float,
    obstacles: np.ndarray,
    safe_distance: float,
    goal_weight: float,
    speed_weight: float,
    obstacle_weight: float,
) -> tuple[float, np.ndarray]:
    trajectory = simulate_trajectory(x, y, theta, v, omega, predict_time, dt)
    if trajectory.shape[0] == 0:
        return float("-inf"), trajectory
    goal_score = goal_weight * evaluate_goal_cost(trajectory, goal_x, goal_y)
    speed_score = speed_weight * v
    obstacle_score = -obstacle_weight * evaluate_obstacle_cost(trajectory, obstacles, safe_distance)
    total = goal_score + speed_score + obstacle_score
    return total, trajectory


def plan_velocity_dwa(
    x: float,
    y: float,
    theta: float,
    goal_x: float,
    goal_y: float,
    obstacles: Sequence[Sequence[float]] | np.ndarray | None,
    _cur_v: float,
    _cur_omega: float,
    cfg: DWAConfig,
) -> tuple[np.ndarray | None, tuple[float, float], bool]:
    """
    Returns (best_trajectory_xytheta, (v, omega), success).
    Mirrors MoveBase3D ``localPlan`` velocity grid search.
    """
    v_min, v_max = cfg.v_min, cfg.v_max
    omega_min, omega_max = cfg.omega_min, cfg.omega_max
    predict_time = cfg.predict_time
    dt = cfg.dt
    safe_distance = cfg.safe_distance
    goal_weight = cfg.goal_weight
    speed_weight = cfg.speed_weight
    obstacle_weight = cfg.obstacle_weight

    v_samples = np.arange(max(v_min, 0.0), min(v_max, cfg.v_cap), cfg.v_sample_step)
    omega_samples = np.arange(max(omega_min, -cfg.omega_cap), min(omega_max, cfg.omega_cap), cfg.omega_sample_step)
    v_grid, omega_grid = np.meshgrid(v_samples, omega_samples)
    v_flat = v_grid.flatten()
    omega_flat = omega_grid.flatten()

    obs_arr = np.zeros((0, 2), dtype=np.float64)
    if obstacles is not None:
        obs_arr = np.asarray(obstacles, dtype=np.float64).reshape(-1, 2) if len(obstacles) else obs_arr

    eval_func = partial(
        evaluate_one_trajectory,
        x=x,
        y=y,
        theta=theta,
        predict_time=predict_time,
        dt=dt,
        goal_x=goal_x,
        goal_y=goal_y,
        obstacles=obs_arr,
        safe_distance=safe_distance,
        goal_weight=goal_weight,
        speed_weight=speed_weight,
        obstacle_weight=obstacle_weight,
    )

    results: list[tuple[float, np.ndarray]] = []
    with ThreadPoolExecutor(max_workers=max(1, cfg.thread_workers)) as ex:
        futures = [ex.submit(eval_func, float(v), float(w)) for v, w in zip(v_flat, omega_flat)]
        for fu in futures:
            try:
                results.append(fu.result())
            except Exception:
                continue

    if not results:
        return None, (0.0, 0.0), False

    best_idx = int(np.argmax([r[0] for r in results]))
    best_score, best_traj = results[best_idx]
    _ = best_score
    best_control = (float(v_flat[best_idx]), float(omega_flat[best_idx]))
    ok = best_traj is not None and len(best_traj) > 0
    return (np.array(best_traj) if ok else None), best_control, ok


def snap_obstacles_xy(points_xy: Iterable[tuple[float, float]], cell: float) -> np.ndarray:
    """Same discretization idea as MoveBase3D obs_cb (floor to grid)."""
    acc: set[tuple[float, float]] = set()
    for px, py in points_xy:
        qx = math.floor(px / cell) * cell
        qy = math.floor(py / cell) * cell
        acc.add((qx, qy))
    if not acc:
        return np.zeros((0, 2), dtype=np.float64)
    return np.array(sorted(acc), dtype=np.float64)
