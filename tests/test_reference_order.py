"""Offline oracle regression: explicit timestamp/payload expectations, NumPy only."""
from pathlib import Path
import sys,unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from map_geometry import paired_reference,nearest_indices,REFERENCE_POLICY

def fixture(headers):
    h=np.asarray(headers,dtype=np.int64)
    xyz=np.column_stack([np.full(len(h),55.805),37.425+np.arange(len(h))*1e-5,np.full(len(h),173.)])
    rover=xyz.copy();rover[:,1]+=.000199
    return dict(master_fix_ht=h.copy(),rover_fix_ht=h.copy(),master_fix_data=xyz,rover_fix_data=rover)

class ReferenceOrder(unittest.TestCase):
    def test_explicit_nonincreasing_rows(self):
        # Keep rows0,1,4. A lower header and duplicate300ms must not acquire
        # the unrelated coordinates carried by rows2/3.
        z=fixture([100000000,300000000,200000000,300000000,400000000])
        expected={k:v[[0,1,4]].copy() for k,v in z.items()}
        q=np.array([100000000,200000000,300000000,400000000])
        actual=paired_reference(z,q);good=paired_reference(expected,q)
        for k in good:np.testing.assert_array_equal(actual[k],good[k])
        np.testing.assert_array_equal(actual['valid'],[True,False,True,True])

    def test_first_duplicate_wins(self):
        z=fixture([100,100,200]);expected={k:v[[0,2]].copy() for k,v in z.items()}
        q=np.array([100,101,200])
        actual=paired_reference(z,q);good=paired_reference(expected,q)
        np.testing.assert_array_equal(actual['base'],good['base'])

    def test_record_high_not_only_adjacent(self):
        z=fixture([100,500,200,300,400,600]);expected={k:v[[0,1,5]].copy() for k,v in z.items()}
        q=np.array([100,300,500,600]);actual=paired_reference(z,q);good=paired_reference(expected,q)
        np.testing.assert_array_equal(actual['base'],good['base'])

    def test_each_receiver_filtered_independently(self):
        z=fixture([100,200,300,400]);z['master_fix_ht']=np.array([100,300,200,400])
        expected={k:(v[[0,1,3]].copy() if k.startswith('master') else v.copy()) for k,v in z.items()}
        q=np.array([100,200,300,400]);a=paired_reference(z,q);b=paired_reference(expected,q)
        for k in b:np.testing.assert_array_equal(a[k],b[k])

    def test_nearest_later_tie_and_inclusive_tolerance(self):
        idx,delta=nearest_indices(np.array([100000000,200000000]),np.array([150000000]))
        self.assertEqual(idx.tolist(),[1]);self.assertEqual(delta.tolist(),[.05])
        z=fixture([100000000]);q=np.array([49999999,50000000,100000000,150000000,150000001])
        np.testing.assert_array_equal(paired_reference(z,q)['valid'],[False,True,True,True,False])

    def test_empty_receivers_and_queries(self):
        self.assertIsNone(paired_reference(fixture([]),np.array([1],np.int64)))
        for name in ['master','rover']:
            z=fixture([100]);z[name+'_fix_ht']=z[name+'_fix_ht'][:0];z[name+'_fix_data']=z[name+'_fix_data'][:0]
            self.assertIsNone(paired_reference(z,np.array([1],np.int64)))
        r=paired_reference(fixture([100]),np.array([],np.int64))
        self.assertEqual(r['base'].shape,(0,3));self.assertEqual(r['quality'].shape,(0,))

    def test_no_input_mutation(self):
        z=fixture([100,300,200,400]);old={k:v.copy() for k,v in z.items()}
        paired_reference(z,np.array([200]))
        for k in z:np.testing.assert_array_equal(z[k],old[k])

    def test_policy_is_explicit(self):
        self.assertEqual(REFERENCE_POLICY,'fix-monotonic-record-high-v1')

if __name__=='__main__':unittest.main(verbosity=2)
