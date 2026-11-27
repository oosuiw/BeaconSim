"""STEP 조립품 → 부품별 삼각형 메시 (이름 경로, 색 포함). OpenCascade(OCP) 필요: pip install cadquery-ocp

read_step(path) → [{'path': ('최상위', ..., '부품'), 'name': '부품', 'tris': (N,3,3) float32 mm(STEP 원래 단위),
                    'color': (r,g,b) 또는 None}]
조립 위치(location)는 모두 적용된 전역 좌표.
"""
import numpy as np


def read_step(path, deflection=0.6, angle=0.5):
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.Quantity import Quantity_Color
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDF import TDF_Label, TDF_LabelSequence
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS
    from OCP.XCAFDoc import XCAFDoc_ColorTool, XCAFDoc_ColorType, XCAFDoc_DocumentTool

    doc = TDocStd_Document(TCollection_ExtendedString('step'))
    rd = STEPCAFControl_Reader()
    rd.SetNameMode(True)
    rd.SetColorMode(True)
    if rd.ReadFile(path) != IFSelect_RetDone:
        raise RuntimeError(f'STEP 읽기 실패: {path}')
    rd.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    ct = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())

    def name_of(lab):
        a = TDataStd_Name()
        if lab.FindAttribute(TDataStd_Name.GetID_s(), a):
            return a.Get().ToExtString().replace('\n', ' ').strip()
        return ''

    def color_of(lab, shape=None):
        c = Quantity_Color()
        for typ in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
            if XCAFDoc_ColorTool.GetColor_s(lab, typ, c) or (shape is not None and ct.GetColor(shape, typ, c)):
                return (c.Red(), c.Green(), c.Blue())
        return None

    def tessellate(shape, loc_trsf):
        BRepMesh_IncrementalMesh(shape, deflection, False, angle, True)
        out = []
        ex = TopExp_Explorer(shape, TopAbs_FACE)
        while ex.More():
            face = TopoDS.Face_s(ex.Current())
            loc = TopLoc_Location()
            tri = BRep_Tool.Triangulation_s(face, loc)
            if tri is not None:
                tr = loc.Transformation()
                n = tri.NbNodes()
                P = np.empty((n, 3))
                for i in range(1, n + 1):
                    p = tri.Node(i).Transformed(tr)
                    P[i - 1] = (p.X(), p.Y(), p.Z())
                F = np.empty((tri.NbTriangles(), 3), np.int64)
                for i in range(1, tri.NbTriangles() + 1):
                    a, b, c = tri.Triangle(i).Get()
                    F[i - 1] = (a - 1, b - 1, c - 1)
                if face.Orientation() == TopAbs_REVERSED:
                    F = F[:, ::-1]
                out.append(P[F])
            ex.Next()
        if not out:
            return np.zeros((0, 3, 3), np.float32)
        T = np.concatenate(out)
        # 조립 위치 (상위 location 누적)
        M = np.array([[loc_trsf.Value(r, c) for c in range(1, 5)] for r in range(1, 4)])
        return (T @ M[:, :3].T + M[:, 3]).astype(np.float32)

    parts = []

    def walk(lab, loc, path, inherited_color):
        nm = name_of(lab)
        ref = TDF_Label()
        if st.IsReference_s(lab) and st.GetReferredShape_s(lab, ref):
            loc = loc.Multiplied(st.GetLocation_s(lab))
            col = color_of(lab) or color_of(ref) or inherited_color
            walk(ref, loc, path, col)
            return
        col = color_of(lab) or inherited_color
        path = path + (nm,)
        if st.IsAssembly_s(lab):
            kids = TDF_LabelSequence()
            st.GetComponents_s(lab, kids)
            for i in range(1, kids.Length() + 1):
                walk(kids.Value(i), loc, path, col)
            return
        shape = st.GetShape_s(lab)
        tris = tessellate(shape, loc.Transformation())
        if len(tris):
            parts.append({'path': path, 'name': nm, 'tris': tris, 'color': col})

    roots = TDF_LabelSequence()
    st.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), (), None)
    return parts


def find_holes(path, name_substr, rmin=0.5, rmax=5.0):
    """STEP 에서 이름에 name_substr 가 들어간 부품의 원통면(구멍) → [(반지름 mm, 중심 xyz, 축 xyz)] 전역 좌표."""
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.BRepGProp import BRepGProp
    from OCP.GeomAbs import GeomAbs_Cylinder
    from OCP.GProp import GProp_GProps
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDF import TDF_Label, TDF_LabelSequence
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    doc = TDocStd_Document(TCollection_ExtendedString('step'))
    rd = STEPCAFControl_Reader()
    rd.SetNameMode(True)
    rd.ReadFile(path)
    rd.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    def nm(lab):
        a = TDataStd_Name()
        return a.Get().ToExtString().replace('\n', ' ') if lab.FindAttribute(TDataStd_Name.GetID_s(), a) else ''
    shapes = []

    def walk(lab, loc):
        ref = TDF_Label()
        if st.IsReference_s(lab) and st.GetReferredShape_s(lab, ref):
            walk(ref, loc.Multiplied(st.GetLocation_s(lab)))
            return
        if name_substr in nm(lab):
            shapes.append(st.GetShape_s(lab).Moved(loc))
            return
        if st.IsAssembly_s(lab):
            kids = TDF_LabelSequence()
            st.GetComponents_s(lab, kids)
            for i in range(1, kids.Length() + 1):
                walk(kids.Value(i), loc)
    roots = TDF_LabelSequence()
    st.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location())
    out = []
    for sh in shapes:
        ex = TopExp_Explorer(sh, TopAbs_FACE)
        while ex.More():
            f = TopoDS.Face_s(ex.Current())
            s = BRepAdaptor_Surface(f)
            if s.GetType() == GeomAbs_Cylinder and rmin <= s.Cylinder().Radius() <= rmax:
                g = GProp_GProps()
                BRepGProp.SurfaceProperties_s(f, g)
                c, a = g.CentreOfMass(), s.Cylinder().Axis().Direction()
                out.append((s.Cylinder().Radius(), np.array([c.X(), c.Y(), c.Z()]), np.array([a.X(), a.Y(), a.Z()])))
            ex.Next()
    return out
