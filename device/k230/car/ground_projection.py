"""Coarse tabletop projection for approach; not grasp pose estimation."""
import math
try:
    from ..pick.pose import undistort, project
except ImportError:
    from pose import undistort, project


class GroundProjection:
    def __init__(self, camera_matrix, distortion, Base_from_Camera, table_plane):
        self.K, self.D, self.T = camera_matrix, distortion, Base_from_Camera
        normal = table_plane['normal']
        length = math.sqrt(sum(v * v for v in normal))
        if not math.isfinite(length) or length < 1e-8:
            raise ValueError('Invalid table plane')
        self.normal = [v / length for v in normal]
        self.offset = table_plane['offset_mm'] / length
        self.origin = [row[3] for row in self.T[:3]]

    def base_to_pixel(self, point):
        # Inverse of the calibrated rigid Camera->Base transform.
        delta = [point[k] - self.origin[k] for k in range(3)]
        camera = [sum(self.T[k][j] * delta[k] for k in range(3)) for j in range(3)]
        return project(camera, self.K, self.D)

    def pixel_to_plane(self, pixel, height_mm=0):
        if not all(math.isfinite(v) for v in list(pixel) + [height_mm]):
            raise ValueError('Invalid pixel or height')
        ray = undistort(pixel, self.K, self.D) + [1.]
        direction = [sum(row[j] * ray[j] for j in range(3)) for row in self.T[:3]]
        denominator = sum(a * b for a, b in zip(self.normal, direction))
        if abs(denominator) < 1e-8:
            raise ValueError('Ray parallel to table')
        distance = (self.offset + height_mm - sum(a * b for a, b in
                    zip(self.normal, self.origin))) / denominator
        if distance <= 0:
            raise ValueError('Table intersection behind camera')
        return [self.origin[k] + distance * direction[k] for k in range(3)]

    def blob_geometry(self, bbox, edge_mm=20):
        """Use the bottom of the blob as a contact proxy, never an exact TCP."""
        x, y, w, h = bbox
        contact = self.pixel_to_plane([x + (w - 1) / 2, y + h - 1])
        above = [contact[k] + edge_mm * self.normal[k] for k in range(3)]
        bottom_pixel, top_pixel = self.base_to_pixel(contact), self.base_to_pixel(above)
        expected_height = abs(bottom_pixel[1] - top_pixel[1])
        if expected_height < 1:
            raise ValueError('Projected cube height too small')
        return {'ground_contact_proxy_Base_mm': contact,
                'expected_vertical_height_px': expected_height,
                'observed_to_expected_height_ratio': h / expected_height}

    def camera_table_foot(self):
        """Orthogonal projection of calibrated camera optical center to table."""
        height = sum(a * b for a, b in zip(self.normal, self.origin)) - self.offset
        return [self.origin[k] - height * self.normal[k] for k in range(3)]

    def table_distance(self, bottom_center):
        """Planar distance from camera foot; caller declares endpoint accuracy."""
        if len(bottom_center) != 3 or not all(math.isfinite(v) for v in bottom_center):
            raise ValueError('Invalid bottom center')
        foot = self.camera_table_foot()
        delta = [bottom_center[k] - foot[k] for k in range(3)]
        off_plane = sum(a * b for a, b in zip(self.normal, delta))
        if abs(off_plane) > 0.1:
            raise ValueError('Bottom center must be on the calibrated table')
        return math.sqrt(sum((delta[k] - off_plane * self.normal[k]) ** 2 for k in range(3)))

    def estimated_bottom_from_image_center(self, pixel, edge_mm=20):
        """Coarse demo only: assume image center represents center at 10mm height.

        A blob centroid is not a measured geometric cube center. Do not expose
        this estimate as a successful fine localization or a grasp coordinate.
        """
        if not math.isfinite(edge_mm) or edge_mm <= 0:
            raise ValueError('Invalid cube size')
        center = self.pixel_to_plane(pixel, height_mm=edge_mm / 2)
        return [center[k] - self.normal[k] * edge_mm / 2 for k in range(3)]
