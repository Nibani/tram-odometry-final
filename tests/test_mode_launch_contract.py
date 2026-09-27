"""ROS-free execution of launch defaults/forwarding with dependency stubs."""
import importlib.util,json,sys,tempfile,types
from pathlib import Path

def main():
    root=Path(__file__).resolve().parents[1]
    defaults={};nodes=[]
    class Argument:
        def __init__(self,name,default_value,**kw):defaults[name]=default_value
    class Configuration:
        def __init__(self,name):self.name=name
        def perform(self,context):return context[self.name]
    class Action:
        def __init__(self,**kwargs):nodes.append(kwargs)
    def module(name,**attrs):
        m=types.ModuleType(name);m.__dict__.update(attrs);sys.modules[name]=m
    module('launch',LaunchDescription=lambda x:x)
    module('launch.actions',DeclareLaunchArgument=Argument,OpaqueFunction=lambda **kw:kw)
    module('launch.substitutions',LaunchConfiguration=Configuration)
    module('launch_ros');module('launch_ros.actions',Node=Action)
    module('ament_index_python')
    with tempfile.TemporaryDirectory() as td:
        share=Path(td);(share/'config').mkdir()
        (share/'config/map_calibration.json').write_text(json.dumps({'enu_to_map_R':[[1,0,0],[0,1,0],[0,0,1]],'enu_to_map_translation':[0,0],'map_z_minus_enu_up':0}))
        module('ament_index_python.packages',get_package_share_directory=lambda _:str(share))
        spec=importlib.util.spec_from_file_location('mode_launch',root/'src/reserve_odometry/launch/odometry.launch.py')
        launch=importlib.util.module_from_spec(spec);spec.loader.exec_module(launch);launch.generate_launch_description()
        expected={'offmap_departure':10.,'offmap_angle':10.,'offmap_min_span':5.,'offmap_horizon':3.,'offmap_max_distance':100.,'adhesion_accel_limit':0.,'adhesion_decel_limit':3.2,'slip_release_rate':.2,'slip_release_hold':3.,'slip_max_latch':10.}
        launch.start(dict(defaults));actual=nodes[-1]['parameters'][0]
        assert {k:actual[k] for k in expected}==expected
        overrides={'offmap_departure':0.,'offmap_angle':12.,'offmap_min_span':6.,'offmap_horizon':4.,'offmap_max_distance':80.,'adhesion_accel_limit':1.65,'adhesion_decel_limit':3.5,'slip_release_rate':.3,'slip_release_hold':2.,'slip_max_latch':8.}
        context=dict(defaults);context.update({k:str(v) for k,v in overrides.items()});launch.start(context)
        assert {k:nodes[-1]['parameters'][0][k] for k in overrides}==overrides
    node=(root/'src/reserve_odometry/src/odometry_node.cpp').read_text()
    for name in expected:assert 'c.'+name+'=declare_parameter<double>("'+name+'",' in node,name
    assert 'c.offmap_departure=declare_parameter<double>("offmap_departure",0.)' in node
    old='us,double(out.route),out.route_s,out.distance,out.init_rms,double(out.gnss_accepted),double(out.gnss_rejected),out.gnss_last_innovation'
    assert 'tm.data={'+old+',double(out.flags),out.slip?1.:0.,out.slip_ratio,double(out.slip_events)}' in node
    assert 'create_publisher<std_msgs::msg::UInt8>("/result/slip_status",10)' in node
    assert 'create_publisher<std_msgs::msg::UInt16>("/result/state_status",10)' in node
    print(json.dumps({'status':'PASS_STATIC_LAUNCH_CONTRACT','defaults':expected,'overrides':overrides,'diagnostics_flags_index':8}))
if __name__=='__main__':main()
