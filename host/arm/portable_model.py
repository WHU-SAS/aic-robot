"""Export the same URDF/mesh data for PC and K230 portable arm arithmetic."""
from pathlib import Path


def export_model(root, mapping, urdf_path=None):
    """Preserve the deployed model's ordering, units and mesh hull vertices."""
    import numpy as np
    from scipy.spatial import ConvexHull
    from host.arm.blue_pick_geometry import PickGeometry

    root = Path(root)
    geometry = PickGeometry(root, mapping, urdf_path=urdf_path)
    model = {'joints': [], 'points': {}}
    for i in range(1, 7):
        item = dict(mapping.joint(i))
        joint = geometry.fk.joints[item['urdf_joint_name']]
        origin = np.eye(4)
        origin[:3, :3] = geometry.fk._rpy_to_matrix(joint['rpy'])
        origin[:3, 3] = joint['xyz']
        item.update(origin_transform=origin.tolist(), axis=joint['axis'].tolist(),
                    child=joint['child'])
        model['joints'].append(item)
    for link, vertices in geometry.vertices.items():
        model['points'][link] = {
            'tip': geometry.tip[link].tolist(),
            'distal': geometry.distal[link].tolist(),
            'hull': vertices[ConvexHull(vertices).vertices].tolist(),
        }
    return model
