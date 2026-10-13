"""Offline regressions for new groups and SD export bookkeeping."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'k230'),str(ROOT/'scripts')]
import collect_raw_pair as capture
from offline_check import verify_raw_pairs

class StorageTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name)
        self.pairs=self.root/'k230/grasp_pairs'
        self.pairs.mkdir(parents=True)
    def tearDown(self): self.temp.cleanup()
    def fixture(self,count=1,pending_last=False):
        groups=[]
        for n in range(1,count+1):
            folder=self.pairs/f'pair_{n:02d}';folder.mkdir()
            pending=pending_last and n==count
            image=b'fixture image '+str(n).encode()
            record={'raw_only':True,'samples_before':[{}]*3,'samples_after':[{}]*3,
                    'camera_capture':{'sha256':hashlib.sha256(image).hexdigest(),'downloaded':not pending}}
            (folder/'record.json').write_text(json.dumps(record))
            if not pending:(folder/'frame.jpg').write_bytes(image)
            groups.append({'group':n,'folder':folder.name,'image_storage':'old pending label'})
        collection={'groups':groups,'current_calibration_baseline':'keep-this-reference'}
        (self.pairs/'collection.json').write_text(json.dumps(collection))
        return collection
    def test_more_than_ten_groups_is_valid(self):
        c=self.fixture(11)
        self.assertEqual(verify_raw_pairs(self.root,c),{'verified':11,'pending':0})
    def test_sd_only_group_is_pending_not_missing_export(self):
        c=self.fixture(11,pending_last=True)
        self.assertEqual(verify_raw_pairs(self.root,c),{'verified':10,'pending':1})
    def test_exported_file_missing_is_rejected(self):
        c=self.fixture();(self.pairs/'pair_01/frame.jpg').unlink()
        with self.assertRaisesRegex(RuntimeError,'Exported raw image missing'):
            verify_raw_pairs(self.root,c)
    def test_storage_index_updates_without_losing_baseline(self):
        self.fixture(2,pending_last=True)
        self.assertEqual(capture.sync_collection_storage(self.pairs),{'downloaded':1,'pending':1})
        c=json.loads((self.pairs/'collection.json').read_text())
        self.assertIn('Computer',c['groups'][0]['image_storage'])
        self.assertIn('pending',c['groups'][1]['image_storage'])
        self.assertEqual(c['current_calibration_baseline'],'keep-this-reference')
    def test_no_pending_download_never_opens_camera(self):
        self.fixture()
        original=capture.sync_collection_storage
        with patch.object(capture,'PAIRS',self.pairs),patch.object(capture,'CameraLink') as camera,patch.object(capture,'sync_collection_storage',side_effect=lambda: original(self.pairs)):
            capture.download_pending()
            camera.assert_not_called()

if __name__=='__main__':unittest.main()
