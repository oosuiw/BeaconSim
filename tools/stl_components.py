"""STL을 연결 성분으로 나눠 라벨 배열을 저장한다 (분석/메시 생성 공용)."""
import numpy as np, sys
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

def load_binary_stl(path):
    f = open(path, 'rb').read()
    n = int(np.frombuffer(f[80:84], '<u4')[0])
    d = np.frombuffer(f[84:84 + n * 50], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return d['v'].reshape(-1, 3, 3).astype(np.float64)

def components(tris):
    V = tris.reshape(-1, 3)
    u, inv = np.unique(np.round(V, 3), axis=0, return_inverse=True)
    inv = inv.reshape(-1, 3)
    rows = np.concatenate([inv[:, 0], inv[:, 1]]); cols = np.concatenate([inv[:, 1], inv[:, 2]])
    g = coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(len(u), len(u)))
    nc, lab = connected_components(g, directed=False)
    return lab[inv[:, 0]]

if __name__ == '__main__':
    tris = load_binary_stl(sys.argv[1])
    lab = components(tris)
    np.save(sys.argv[2], lab)
    print('components', lab.max() + 1)
