"""Actual Humble/DDS mode visibility test; synthetic branch/outlier, no reference.

python3 tests/ros_mode_status.py --out results/ci_mode_status
Uses ros_checker_compat geometry/process helpers. Four nodes share18s input.
Static launch argument forwarding is separately tested by test_mode_launch_contract.py.
"""
import argparse,hashlib,importlib.util,json,math,os,subprocess,time,traceback
from pathlib import Path

def run(args):
    os.environ['ROS_DOMAIN_ID']=str(args.domain)
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import UInt8,UInt16,Float64MultiArray
    from rcl_interfaces.srv import GetParameters
    from tram_vehicle_msgs.msg import DriverControllerCommand,VelocitySensor
    spec=importlib.util.spec_from_file_location('mode_helpers',args.helpers)
    helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
    args.out.mkdir(parents=True,exist_ok=True)
    cases=['mode_bridge','mode_disabled','mode_outlier','mode_conflict'];epoch=100_000_000_000;dt=50_000_000
    samples={c:{'velocity':[],'position':[],'slip':[],'state':[],'diagnostics':[]} for c in cases}
    checks=[];processes=[];logs=[];node=None;current=[epoch];start=time.monotonic()
    report={'status':'NOT_COMPLETED','synthetic_only':True,'reference_used':False,'checks':checks,'commands':{},'parameters':{},'source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'helper_sha256':hashlib.sha256(args.helpers.read_bytes()).hexdigest()}
    def check(name,ok,evidence=None):checks.append({'name':name,'pass':bool(ok),'evidence':evidence})
    def elapsed(stamp):return (stamp-epoch)*1e-9
    def spin(deadline):
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=min(.003,max(0.,deadline-time.monotonic())))
    def pose(c,m):
        p=m.pose.pose.position;samples[c]['position'].append({'stamp':helper.stamp_ns(m.header.stamp),'xyz':[p.x,p.y,p.z],'cov':[m.pose.covariance[i] for i in [0,7,14]]})
    try:
        rclpy.init();node=Node('mode_status_fixture')
        route=args.out/'line.csv';route.write_text('s,x,y,z\n0,0,0,0\n1000,1000,0,0\n')
        calibration={'enu_to_map_R':[[1,0,0],[0,1,0],[0,0,1]],'enu_to_map_translation':[0,0],'map_z_minus_enu_up':0,'origin_lla':[55.805,37.425,170.]}
        clock=node.create_publisher(Clock,'/clock',10)
        wheels=[node.create_publisher(VelocitySensor,'/compat/input/'+side,10) for side in ['front','rear']]
        controller=node.create_publisher(DriverControllerCommand,'/compat/input/command',10)
        fixes={(c,k):node.create_publisher(NavSatFix,'/compat/'+c+'/'+k,10) for c in cases for k in ['master','rover']}
        clients={}
        for c in cases:
            node.create_subscription(VelocitySensor,'/compat/'+c+'/velocity',lambda m,c=c:samples[c]['velocity'].append({'stamp':helper.stamp_ns(m.header.stamp),'value':m.velocity}),1000)
            node.create_subscription(Odometry,'/compat/'+c+'/position',lambda m,c=c:pose(c,m),1000)
            node.create_subscription(UInt8,'/compat/'+c+'/slip_status',lambda m,c=c:samples[c]['slip'].append({'clock':current[0],'flags':int(m.data)}),1000)
            node.create_subscription(UInt16,'/compat/'+c+'/state_status',lambda m,c=c:samples[c]['state'].append({'clock':current[0],'flags':int(m.data)}),1000)
            node.create_subscription(Float64MultiArray,'/compat/'+c+'/diagnostics',lambda m,c=c:samples[c]['diagnostics'].append(list(m.data)),200)
            p=helper.parameters(calibration,route.resolve(),c)
            p.update(initial_gnss_seconds=3.,gnss_xy_residual_correction=True,body_velocity_output=False,offmap_departure=0. if c=='mode_disabled' else 10.,offmap_angle=10.,offmap_min_span=5.,offmap_horizon=3.,offmap_max_distance=20.)
            command=helper.command(p,c)+['-r','/result/state_status:=/compat/'+c+'/state_status']
            report['commands'][c]=command;report['parameters'][c]={k:p[k] for k in p if k.startswith('offmap_')}
            log=(args.out/(c+'.log')).open('w');logs.append(log)
            processes.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,preexec_fn=helper.child_setup))
            clients[c]=node.create_client(GetParameters,'/checker_compat_'+c+'/get_parameters')
        deadline=time.monotonic()+args.discovery_timeout;publishers=[clock,*wheels,controller,*fixes.values()]
        while time.monotonic()<deadline:
            spin(time.monotonic()+.02)
            if all(p.get_subscription_count()>=1 for p in publishers) and all(c.service_is_ready() for c in clients.values()):break
            if any(p.poll() is not None for p in processes):raise RuntimeError('Node exited before discovery')
        ready=all(p.get_subscription_count()>=1 for p in publishers) and all(c.service_is_ready() for c in clients.values())
        check('DDS_and_parameter_services',ready)
        if not ready:raise RuntimeError('Discovery timeout')
        for c,client in clients.items():
            request=GetParameters.Request();request.names=list(report['parameters'][c]);future=client.call_async(request);deadline=time.monotonic()+2.
            while not future.done() and time.monotonic()<deadline:spin(time.monotonic()+.01)
            actual={k:v.double_value for k,v in zip(request.names,future.result().values)} if future.done() and future.result() else {}
            check(c+'_declared_parameter_values',actual==report['parameters'][c],actual)
        def publish_fix(c,k,ns,point):
            msg=NavSatFix();helper.set_stamp(msg.header.stamp,ns);msg.status.status=0
            msg.latitude,msg.longitude,msg.altitude=helper.enu_to_lla(point,calibration['origin_lla']);fixes[c,k].publish(msg)
        wall=time.monotonic()
        for i in range(361):
            spin(wall+i*.05);ns=epoch+i*dt;t=i*.05;current[0]=ns
            clk=Clock();helper.set_stamp(clk.clock,ns);clock.publish(clk);spin(time.monotonic()+.006)
            for p in wheels:
                msg=VelocitySensor();helper.set_stamp(msg.header.stamp,ns);msg.velocity=18.;p.publish(msg)
            if i%2==0:
                for c in cases:
                    if i<=56:
                        for k,x in [('master',-9.873),('rover',2.563)]:publish_fix(c,k,ns,[50.+5*t+x,0.,3.])
                    elif c in ['mode_bridge','mode_disabled'] and 80<=i<=220:
                        d=5*(t-4)+2.563;publish_fix(c,'rover',ns,[70.+d*math.cos(math.pi/6),d*math.sin(math.pi/6),3.])
                    elif c=='mode_outlier' and 80<=i<=100:publish_fix(c,'rover',ns,[50.+5*t+2.563,12. if i==100 else 0.,3.])
                    elif c=='mode_conflict' and 80<=i<=160:
                        publish_fix(c,'master',ns,[50.+5*t-9.873,0.,3.]);publish_fix(c,'rover',ns,[50.+5*t+2.563,12.,3.])
            spin(time.monotonic()+.006)
            cmd=DriverControllerCommand();helper.set_stamp(cmd.header.stamp,ns);cmd.position=0;controller.publish(cmd)
            spin(time.monotonic()+.003)
            if any(p.poll() is not None for p in processes):raise RuntimeError('Node exited during fixture')
        spin(time.monotonic()+.2)
        for c,s in samples.items():
            v=s['velocity'];poses=s['position'];states=s['state'];slips=s['slip'];diags=s['diagnostics']
            check(c+'_finite_20hz_coverage',len(v)>=350 and all(math.isfinite(r['value']) and abs(r['value']-5.)<1e-6 for r in v),len(v))
            check(c+'_strict_stamps',all(b['stamp']>a['stamp'] for a,b in zip(v,v[1:])))
            check(c+'_both_status_topics',len(states)>=350 and len(slips)>=350,[len(states),len(slips)])
            check(c+'_legacy_low8_compatibility',all(r['flags']==0 for r in slips if elapsed(r['clock'])>=3.3) and all((r['flags']&255)==0 for r in states if elapsed(r['clock'])>=3.3))
            check(c+'_diagnostic_prefix_and_fullflags',len(diags)>=16 and all(len(r)>=9 and r[1]==0 and all(math.isfinite(x) for x in r) and r[8]==int(r[8]) for r in diags),len(diags))
            check(c+'_distance_index3_preserved',len(diags)>1 and all(b[3]>a[3] for a,b in zip(diags,diags[1:])) and diags[-1][3]>80.,[r[3] for r in diags])
            check(c+'_normal_covariance_below_mode_floor',any(all(x<100. for x in r['cov']) for r in poses if 3.2<elapsed(r['stamp'])<3.8))
        for bit,name,c,floor in [(256,'bridge','mode_bridge',100.),(512,'returning','mode_bridge',100.),(1024,'pair_conflict','mode_conflict',625.)]:
            states=samples[c]['state'];poses=samples[c]['position'];diags=samples[c]['diagnostics']
            flagged=[r for r in states if r['flags']&bit]
            matched=[]
            for p in poses:
                nearby=[s for s in states if abs(p['stamp']-s['clock'])<=100_000_000]
                if len(nearby)>=3 and all(s['flags']&bit for s in nearby):matched.append(p)
            check(name+'_full_UInt16_flag',bool(flagged),len(flagged));check(name+'_diagnostics_index8',any(int(r[8])&bit for r in diags if len(r)>=9))
            check(name+'_heuristic_covariance_floor',bool(matched) and all(all(x>=floor for x in p['cov']) for p in matched),{'matched':len(matched),'floor_variance_m2':floor})
        for c in ['mode_disabled','mode_outlier']:
            check(c+'_no_high_mode_flags',all(not (r['flags']&(256|512|1024)) for r in samples[c]['state']))
        errors=[math.dist(p['xyz'],[50.+5*elapsed(p['stamp']),0.,0.]) for p in samples['mode_outlier']['position']]
        check('isolated12m_outlier_not_amplified',bool(errors) and max(errors)<=3.01,max(errors,default=None))
        report['status']='PASS' if all(x['pass'] for x in checks) else 'FAIL'
    except Exception:
        report['status']='ERROR';report['exception']=traceback.format_exc()
    finally:
        for p in processes:helper.stop(p)
        for log in logs:log.close()
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        report['seconds']=time.monotonic()-start;report['coverage']={c:{k:len(v) for k,v in s.items()} for c,s in samples.items()}
        (args.out/'mode_status.json').write_text(json.dumps(helper.json_safe(report),indent=2,allow_nan=False))
        (args.out/'samples.json').write_text(json.dumps(helper.json_safe(samples),indent=2,allow_nan=False))
    print(json.dumps({'status':report['status'],'out':str(args.out),'seconds':report['seconds']}));return 0 if report['status']=='PASS' else 1

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--helpers',type=Path,default=Path(__file__).with_name('ros_checker_compat.py'));p.add_argument('--domain',type=int,default=64);p.add_argument('--discovery-timeout',type=float,default=8.);raise SystemExit(run(p.parse_args()))
