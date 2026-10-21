"""Bundle board modules for the V0 USBDBG adapter without rewriting code literals."""
import ast
from host.project_paths import ROOT
from host.vision.config import load_vision_config

MODULES = ('blue_face_core', 'blue_detector', 'camera', 'detect_blue_faces')


def module_source(name):
    source = (ROOT / 'device/k230/vision' / (name + '.py')).read_text()
    lines = source.splitlines(keepends=True)
    remove = set()
    for node in ast.parse(source).body:
        bundled_import = isinstance(node, ast.ImportFrom) and node.module in MODULES
        main_guard = (isinstance(node, ast.If) and isinstance(node.test, ast.Compare)
                      and isinstance(node.test.left, ast.Name) and node.test.left.id == '__name__')
        if bundled_import or main_guard:
            remove.update(range(node.lineno - 1, node.end_lineno))
    return ''.join(line for index, line in enumerate(lines) if index not in remove)


def build_capture_script(config=None, preview=False, debug_image_remote=None,
                         local_roi_remote=None, debug_image_roi=False):
    config = config if config is not None else load_vision_config()
    source = '\n'.join(module_source(name) for name in MODULES)
    return source + '\nrun_capture(%r, %r, %r, %r, %r)\n' % (
        config, preview, debug_image_remote, local_roi_remote, debug_image_roi)
