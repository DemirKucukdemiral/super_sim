"""
Terrain Elevation and Surface Model for 14-DoF Vehicle Dynamics.
Computes ground elevation z(x, y), surface gradients, road grade (pitch),
camber (roll), and 4-corner wheel heights for flat and non-flat (hilly) environments.
Also generates matching 3D OBJ terrain meshes for Gazebo Harmonic.
"""

import math
from enum import Enum


class TerrainType(Enum):
    FLAT = "FLAT"
    HILLS = "HILLS"
    ROLLER_COASTER = "ROLLER_COASTER"
    SLOPED_RAMP = "SLOPED_RAMP"


def smoothstep(edge0: float, edge1: float, x: float) -> float:
    """Standard Hermite smooth interpolation between 0 and 1."""
    t = max(0.0, min(1.0, (x - edge0) / max(1e-5, edge1 - edge0)))
    return t * t * (3.0 - 2.0 * t)


class TerrainModel:
    def __init__(self, terrain_type: TerrainType = TerrainType.HILLS):
        self.terrain_type = terrain_type

    def set_type(self, terrain_type: TerrainType):
        self.terrain_type = terrain_type

    def get_height(self, x: float, y: float) -> float:
        """Returns the ground elevation z [m] at world coordinate (x, y)."""
        if self.terrain_type == TerrainType.FLAT:
            return 0.0

        elif self.terrain_type == TerrainType.HILLS:
            # Flat starting paddock for vehicle spawn / staging
            if x <= 15.0:
                return 0.0
            
            z = 0.0
            # Hill 1: Crest (+3.5m) between x=15 and x=95 (peak at x=55m)
            if 15.0 < x <= 95.0:
                phase = (x - 15.0) / 80.0
                z = 3.5 * (math.sin(phase * math.pi) ** 2)
            # Dip 1: Valley (-1.8m) between x=95 and x=165 (trough at x=130m)
            elif 95.0 < x <= 165.0:
                phase = (x - 95.0) / 70.0
                z = -1.8 * (math.sin(phase * math.pi) ** 2)
            # Hill 2: Mountain Ridge (+4.8m) with lateral camber between x=165 and x=265
            elif 165.0 < x <= 265.0:
                phase = (x - 165.0) / 100.0
                lateral_factor = 1.0 + 0.15 * math.sin(y * 0.08)
                z = 4.8 * (math.sin(phase * math.pi) ** 2) * lateral_factor
            # Gentle undulating return section
            elif 265.0 < x <= 330.0:
                phase = (x - 265.0) / 65.0
                z = 1.8 * math.sin(phase * math.pi) * (1.0 - smoothstep(265.0, 330.0, x))
            else:
                z = 0.0

            # Lateral roll-off: terrain gently tapers to flat far outside the track
            lat_weight = 1.0 - smoothstep(60.0, 120.0, abs(y))
            return z * lat_weight

        elif self.terrain_type == TerrainType.ROLLER_COASTER:
            # Intense sinusoidal rolling hills along X
            if x <= 15.0:
                return 0.0
            blend = smoothstep(15.0, 30.0, x)
            # Wavelength 50m, Amplitude 2.8m (steep 14% peak grade)
            wavelength = 50.0
            z_wave = 2.8 * math.sin((x - 20.0) * (2.0 * math.pi / wavelength))
            lat_weight = 1.0 - smoothstep(20.0, 60.0, abs(y))
            return z_wave * blend * lat_weight

        elif self.terrain_type == TerrainType.SLOPED_RAMP:
            # Flat from 0 to 20m, then rises steadily at 10% slope (5.7 deg)
            if x <= 20.0:
                return 0.0
            blend = smoothstep(20.0, 40.0, x)
            slope_z = (x - 20.0) * 0.10
            return slope_z * blend

        return 0.0

    def get_gradient(self, x: float, y: float, eps: float = 0.1) -> tuple[float, float]:
        """Calculates terrain surface gradient (dz/dx, dz/dy) via central difference."""
        dz_dx = (self.get_height(x + eps, y) - self.get_height(x - eps, y)) / (2.0 * eps)
        dz_dy = (self.get_height(x, y + eps) - self.get_height(x, y - eps)) / (2.0 * eps)
        return dz_dx, dz_dy

    def get_slope_angles(self, x: float, y: float, yaw: float) -> tuple[float, float, float]:
        """
        Calculates ground pitch, roll, and grade percentage aligned with vehicle heading yaw.
        Returns:
            (pitch_rad, roll_rad, grade_pct):
                pitch_rad > 0 : vehicle is pointed uphill (nose up)
                roll_rad > 0  : vehicle is tilted left-up (camber)
                grade_pct     : vertical rise over horizontal run in %
        """
        dz_dx, dz_dy = self.get_gradient(x, y)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        # Directional derivative in forward vehicle direction
        directional_slope_fwd = dz_dx * cy + dz_dy * sy
        # Directional derivative in lateral left vehicle direction
        directional_slope_lat = -dz_dx * sy + dz_dy * cy

        pitch_rad = math.atan(directional_slope_fwd)
        roll_rad = math.atan(directional_slope_lat)
        grade_pct = directional_slope_fwd * 100.0

        return pitch_rad, roll_rad, grade_pct

    def get_wheel_heights(
        self,
        car_x: float,
        car_y: float,
        car_yaw: float,
        a: float = 1.5416,
        b: float = 1.5416,
        wf: float = 0.92825,
        wr: float = 0.92825
    ) -> tuple[float, float, float, float]:
        """
        Calculates ground elevation under each of the 4 tires:
        Returns: (z_fl, z_fr, z_rl, z_rr) [m].
        """
        cy = math.cos(car_yaw)
        sy = math.sin(car_yaw)

        # Front-Left
        x_fl = car_x + a * cy - wf * sy
        y_fl = car_y + a * sy + wf * cy
        z_fl = self.get_height(x_fl, y_fl)

        # Front-Right
        x_fr = car_x + a * cy + wf * sy
        y_fr = car_y + a * sy - wf * cy
        z_fr = self.get_height(x_fr, y_fr)

        # Rear-Left
        x_rl = car_x - b * cy - wr * sy
        y_rl = car_y - b * sy + wr * cy
        z_rl = self.get_height(x_rl, y_rl)

        # Rear-Right
        x_rr = car_x - b * cy + wr * sy
        y_rr = car_y - b * sy - wr * cy
        z_rr = self.get_height(x_rr, y_rr)

        return z_fl, z_fr, z_rl, z_rr

    def export_obj_mesh(
        self,
        output_filepath: str,
        x_min: float = -80.0,
        x_max: float = 380.0,
        y_min: float = -160.0,
        y_max: float = 160.0,
        step_x: float = 2.5,
        step_y: float = 2.5
    ):
        """Exports 3D OBJ terrain mesh with UV coordinates and asphalt normals."""
        xs = []
        x = x_min
        while x <= x_max + 1e-4:
            xs.append(round(x, 3))
            x += step_x

        ys = []
        y = y_min
        while y <= y_max + 1e-4:
            ys.append(round(y, 3))
            y += step_y

        nx = len(xs)
        ny = len(ys)

        with open(output_filepath, "w") as f:
            f.write("# Super Sim 3D Terrain Mesh (Hills & Undulations)\n")
            f.write(f"# Grid: {nx} x {ny} = {nx*ny} vertices\n\n")

            # Write vertices
            for j in range(ny):
                for i in range(nx):
                    vx = xs[i]
                    vy = ys[j]
                    vz = self.get_height(vx, vy)
                    f.write(f"v {vx:.3f} {vy:.3f} {vz:.3f}\n")

            # Write UV texture coordinates
            for j in range(ny):
                for i in range(nx):
                    u = (xs[i] - x_min) / (x_max - x_min) * 20.0
                    v = (ys[j] - y_min) / (y_max - y_min) * 10.0
                    f.write(f"vt {u:.4f} {v:.4f}\n")

            # Write vertex normals
            for j in range(ny):
                for i in range(nx):
                    vx = xs[i]
                    vy = ys[j]
                    gx, gy = self.get_gradient(vx, vy)
                    norm = math.sqrt(gx*gx + gy*gy + 1.0)
                    f.write(f"vn {-gx/norm:.4f} {-gy/norm:.4f} {1.0/norm:.4f}\n")

            f.write("\ns 1\n")
            # Write faces (triangles)
            for j in range(ny - 1):
                for i in range(nx - 1):
                    # 1-indexed vertex IDs
                    v1 = j * nx + i + 1
                    v2 = j * nx + (i + 1) + 1
                    v3 = (j + 1) * nx + (i + 1) + 1
                    v4 = (j + 1) * nx + i + 1

                    # Triangle 1: v1, v2, v3
                    f.write(f"f {v1}/{v1}/{v1} {v2}/{v2}/{v2} {v3}/{v3}/{v3}\n")
                    # Triangle 2: v1, v3, v4
                    f.write(f"f {v1}/{v1}/{v1} {v3}/{v3}/{v3} {v4}/{v4}/{v4}\n")

        print(f"Exported terrain OBJ mesh with {nx*ny} vertices to {output_filepath}")
