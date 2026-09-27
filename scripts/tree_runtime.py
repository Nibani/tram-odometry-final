from __future__ import annotations
import numpy as np
from numba import njit

def export_arrays(model):
    sizes=[len(a[0].nodes) for a in model._predictors];offset=np.r_[0,np.cumsum(sizes)]
    feature=[];threshold=[];left=[];right=[];value=[];leaf=[]
    for k,pred in enumerate(model._predictors):
        n=pred[0].nodes;feature.extend(n['feature_idx']);threshold.extend(n['num_threshold'])
        left.extend(n['left']+offset[k]);right.extend(n['right']+offset[k]);value.extend(n['value']);leaf.extend(n['is_leaf'])
    return (np.array(offset[:-1],dtype=np.int64),np.array(feature,dtype=np.int64),np.array(threshold,dtype=float),
      np.array(left,dtype=np.int64),np.array(right,dtype=np.int64),np.array(value,dtype=float),np.array(leaf,dtype=np.uint8),float(model._baseline_prediction[0,0]))

@njit(cache=True)
def tree_predict_one(x,model):
    offsets,feature,threshold,left,right,value,leaf,bias=model;y=bias
    for root in offsets:
        i=root
        while not leaf[i]:i=left[i] if x[feature[i]]<=threshold[i] else right[i]
        y+=value[i]
    return y

@njit(cache=True)
def tree_predict(X,model):
    y=np.empty(len(X))
    for i in range(len(X)):y[i]=tree_predict_one(X[i],model)
    return y

def load_arrays(path):
    """Load our own flat, non-pickle model; normalize scalar bias for Numba."""
    with np.load(path,allow_pickle=False) as z:
        return tuple(z[f'arr{i}'].copy() for i in range(7))+(float(z['arr7']),)
