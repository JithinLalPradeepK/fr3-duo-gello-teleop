"""Convert the twin URDF (isaac/cache/fr3_duo_twin.urdf, written by scripts/twin_isaac_bridge.py) to USD.
Run with the Isaac Sim venv:  OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim_env/bin/python isaac/convert_twin.py"""
import os
from isaacsim import SimulationApp

HERE = os.path.dirname(os.path.abspath(__file__))
app = SimulationApp({'headless': True})
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig  # noqa: E402

cfg = URDFImporterConfig(urdf_path=os.path.join(HERE, 'cache', 'fr3_duo_twin.urdf'),
                         usd_path=os.path.join(HERE, 'cache', 'usd'), merge_fixed_joints=False, fix_base=True,
                         collision_from_visuals=False, robot_type='Manipulator')
out = URDFImporter(cfg).import_urdf()
print('USD_OUT', out, flush=True)
from pxr import Usd  # noqa: E402
st = Usd.Stage.Open(out)
for p in st.Traverse():
    if p.GetTypeName() in ('Xform', 'Scope') and p.GetPath().pathElementCount <= 4:
        print('PRIM', p.GetPath(), p.GetTypeName())
app.close()
