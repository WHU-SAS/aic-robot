import unittest
from host.car.video_labels import label_lines, label_position


class VideoLabelTests(unittest.TestCase):
    def test_label_uses_current_frame_metrics_and_marks_distance_estimated(self):
        row = {'status': 'tracking', 'bbox_px': [500, 200, 90, 100],
               'center_pixel': [545, 250], 'frame_id': 7, 'processing_ms': 36,
               'pixels': 8100, 'estimated_table_distance_mm': 515.4}
        lines = '\n'.join(label_lines(row))
        for value in ('545, 250', '8100', '515 mm', '估计', '帧 7'):
            self.assertIn(value, lines)
        row['frame_id'] = 8; row['estimated_table_distance_mm'] = 600
        self.assertIn('600 mm', '\n'.join(label_lines(row)))
        for state in ('lost', 'no_target'):
            self.assertEqual(label_lines(dict(row, status=state)), [])

    def test_panel_follows_box_and_stays_inside_canvas_at_edges(self):
        panel, canvas = (440, 106), (1280, 1040)
        for bbox in ([25, 120, 80, 90], [1150, 200, 80, 90], [500, 5, 80, 90], [500, 850, 80, 90]):
            x, y = label_position(bbox, panel, canvas)
            self.assertTrue(0 <= x and x + panel[0] <= canvas[0])
            self.assertTrue(80 <= y and y + panel[1] <= canvas[1])
        self.assertNotEqual(label_position([200, 300, 90, 100], panel, canvas),
                            label_position([600, 500, 90, 100], panel, canvas))


if __name__ == '__main__': unittest.main()
