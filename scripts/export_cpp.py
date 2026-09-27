from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
z=np.load(ROOT/'config/drive_tree_v1.npz')
names=['roots','feature','threshold','left','right','value','leaf']
s=['#pragma once','#include <array>','namespace reserve_odometry::drive {']
for k,name in enumerate(names):
 a=z[f'arr{k}'];typ='double' if k in (2,5) else 'int'
 vals=[format(float(v),'.17g') if typ=='double' else str(int(v)) for v in a]
 s.append(f'inline constexpr std::array<{typ},{len(a)}> {name} = {{{{'+','.join(vals)+'}};')
s.append(f'inline constexpr double bias = {float(z["arr7"]):.17g};')
s.append('''inline double predict(const std::array<double,5>& x) {
 double y=bias;
 for(int root:roots) {int j=root;while(!leaf[j]) j=x[feature[j]]<=threshold[j]?left[j]:right[j];y+=value[j];}
 return y;
}
}''')
p=ROOT/'src/reserve_odometry/include/reserve_odometry/drive_model.hpp';p.write_text('\n'.join(s)+'\n');print(p,len(z['arr0']),'trees',len(z['arr1']),'nodes')
