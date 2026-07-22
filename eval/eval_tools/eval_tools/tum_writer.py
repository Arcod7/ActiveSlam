class TUMWriter:
    """Writes poses in TUM trajectory format: `timestamp tx ty tz qx qy qz qw`,
    one pose per line. Directly consumable by `evo_ape` / `evo_rpe`.
    """

    def __init__(self, filepath: str):
        self._file = open(filepath, 'w')
        self._file.write('# timestamp tx ty tz qx qy qz qw\n')

    def write_pose(self, timestamp: float, position, quaternion):
        x, y, z = position
        qx, qy, qz, qw = quaternion
        self._file.write(
            f'{timestamp:.6f} {x:.6f} {y:.6f} {z:.6f} '
            f'{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n')
        self._file.flush()

    def close(self):
        self._file.close()
