#!/usr/bin/env python3

import json
import sqlite3
from pathlib import Path


DATA_ROOT = Path("/data/sets/nuscenes/can_bus")
DB_PATH = Path("/home/dfw/dev/imu-gps-eskf/tmp/nuscenes_can_bus.sqlite3")


def load_json_array(path: Path) -> list[dict]:
    with path.open() as f:
        return json.load(f)


def create_schema(conn: sqlite3.Connection) -> None:
    conn.executescript(
        """
        DROP TABLE IF EXISTS imu_samples;
        DROP TABLE IF EXISTS pose_samples;
        DROP TABLE IF EXISTS scenes;

        CREATE TABLE scenes (
            scene_name TEXT PRIMARY KEY,
            pose_path TEXT NOT NULL,
            imu_path TEXT NOT NULL
        );

        CREATE TABLE pose_samples (
            scene_name TEXT NOT NULL,
            pose_index INTEGER NOT NULL,
            utime INTEGER NOT NULL,
            pos_x REAL NOT NULL,
            pos_y REAL NOT NULL,
            pos_z REAL NOT NULL,
            orientation_w REAL NOT NULL,
            orientation_x REAL NOT NULL,
            orientation_y REAL NOT NULL,
            orientation_z REAL NOT NULL,
            vel_x REAL NOT NULL,
            vel_y REAL NOT NULL,
            vel_z REAL NOT NULL,
            accel_x REAL NOT NULL,
            accel_y REAL NOT NULL,
            accel_z REAL NOT NULL,
            rotation_rate_x REAL NOT NULL,
            rotation_rate_y REAL NOT NULL,
            rotation_rate_z REAL NOT NULL,
            PRIMARY KEY (scene_name, pose_index),
            FOREIGN KEY (scene_name) REFERENCES scenes(scene_name)
        );

        CREATE TABLE imu_samples (
            scene_name TEXT NOT NULL,
            imu_index INTEGER NOT NULL,
            utime INTEGER NOT NULL,
            linear_accel_x REAL NOT NULL,
            linear_accel_y REAL NOT NULL,
            linear_accel_z REAL NOT NULL,
            rotation_rate_x REAL NOT NULL,
            rotation_rate_y REAL NOT NULL,
            rotation_rate_z REAL NOT NULL,
            q_w REAL NOT NULL,
            q_x REAL NOT NULL,
            q_y REAL NOT NULL,
            q_z REAL NOT NULL,
            PRIMARY KEY (scene_name, imu_index),
            FOREIGN KEY (scene_name) REFERENCES scenes(scene_name)
        );
        """
    )


def create_indexes(conn: sqlite3.Connection) -> None:
    conn.executescript(
        """
        CREATE INDEX pose_samples_scene_utime_idx
        ON pose_samples (scene_name, utime);

        CREATE INDEX imu_samples_scene_utime_idx
        ON imu_samples (scene_name, utime);
        """
    )


def pose_rows(scene_name: str, pose_data: list[dict]) -> list[tuple]:
    rows = []
    for pose_index, sample in enumerate(pose_data):
        rows.append(
            (
                scene_name,
                pose_index,
                sample["utime"],
                sample["pos"][0],
                sample["pos"][1],
                sample["pos"][2],
                sample["orientation"][0],
                sample["orientation"][1],
                sample["orientation"][2],
                sample["orientation"][3],
                sample["vel"][0],
                sample["vel"][1],
                sample["vel"][2],
                sample["accel"][0],
                sample["accel"][1],
                sample["accel"][2],
                sample["rotation_rate"][0],
                sample["rotation_rate"][1],
                sample["rotation_rate"][2],
            )
        )
    return rows


def imu_rows(scene_name: str, imu_data: list[dict]) -> list[tuple]:
    rows = []
    for imu_index, sample in enumerate(imu_data):
        rows.append(
            (
                scene_name,
                imu_index,
                sample["utime"],
                sample["linear_accel"][0],
                sample["linear_accel"][1],
                sample["linear_accel"][2],
                sample["rotation_rate"][0],
                sample["rotation_rate"][1],
                sample["rotation_rate"][2],
                sample["q"][0],
                sample["q"][1],
                sample["q"][2],
                sample["q"][3],
            )
        )
    return rows


def scene_pairs() -> list[tuple[str, Path, Path]]:
    pairs = []
    for pose_path in sorted(DATA_ROOT.glob("scene-????_pose.json")):
        scene_name = pose_path.name.removesuffix("_pose.json")
        imu_path = DATA_ROOT / f"{scene_name}_ms_imu.json"
        if imu_path.is_file():
            pairs.append((scene_name, pose_path, imu_path))
    return pairs


def main() -> None:
    DB_PATH.parent.mkdir(parents=True, exist_ok=True)
    if DB_PATH.exists():
        DB_PATH.unlink()

    pairs = scene_pairs()

    with sqlite3.connect(DB_PATH) as conn:
        conn.execute("PRAGMA journal_mode = OFF;")
        conn.execute("PRAGMA synchronous = OFF;")
        conn.execute("PRAGMA temp_store = MEMORY;")
        conn.execute("PRAGMA foreign_keys = ON;")

        create_schema(conn)

        scene_count = 0
        pose_count = 0
        imu_count = 0

        for scene_name, pose_path, imu_path in pairs:
            pose_data = load_json_array(pose_path)
            imu_data = load_json_array(imu_path)

            conn.execute(
                """
                INSERT INTO scenes (scene_name, pose_path, imu_path)
                VALUES (?, ?, ?)
                """,
                (scene_name, str(pose_path), str(imu_path)),
            )

            pose_batch = pose_rows(scene_name, pose_data)
            imu_batch = imu_rows(scene_name, imu_data)

            conn.executemany(
                """
                INSERT INTO pose_samples (
                    scene_name, pose_index, utime,
                    pos_x, pos_y, pos_z,
                    orientation_w, orientation_x, orientation_y, orientation_z,
                    vel_x, vel_y, vel_z,
                    accel_x, accel_y, accel_z,
                    rotation_rate_x, rotation_rate_y, rotation_rate_z
                )
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                pose_batch,
            )

            conn.executemany(
                """
                INSERT INTO imu_samples (
                    scene_name, imu_index, utime,
                    linear_accel_x, linear_accel_y, linear_accel_z,
                    rotation_rate_x, rotation_rate_y, rotation_rate_z,
                    q_w, q_x, q_y, q_z
                )
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                imu_batch,
            )

            scene_count += 1
            pose_count += len(pose_batch)
            imu_count += len(imu_batch)

        create_indexes(conn)

    print(f"wrote {DB_PATH}")
    print(f"scenes: {scene_count}")
    print(f"pose_samples: {pose_count}")
    print(f"imu_samples: {imu_count}")


if __name__ == "__main__":
    main()
