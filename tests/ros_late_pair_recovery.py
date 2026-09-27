"""Actual-DDS delayed-pair initialization test, synthetic geometry, no reference.

After installation in candidate/tests:
  python3 tests/ros_late_pair_recovery.py --out results/ci_late_pair

One node, eight seconds of 5 m/s wheel/controller input. No GNSS before4s,
then valid paired fixes. Expected world position uses absolute measurement epoch,
so resetting the distance integral or restarting the motion timeline cannot pass.
Requires the explicitly enabled sparse-GNSS initialization-retry branch.
"""
from pathlib import Path
import argparse, hashlib, importlib.util, json, math, os, subprocess, sys, time, traceback

def run(args):
    os.environ['ROS_DOMAIN_ID']=str(args.domain)
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import Float64MultiArray
    from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor
    spec=importlib.util.spec_from_file_location('late_pair_geometry_helpers',args.helpers)
    helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
    args.out.mkdir(parents=True,exist_ok=True)
    calibration=json.loads(args.calibration.read_text());rotation=calibration['enu_to_map_R']
    yaw=math.atan2(rotation[1][0],rotation[0][0]);translation=calibration['enu_to_map_translation']+[calibration['map_z_minus_enu_up']]
    origin=calibration['origin_lla'];epoch=100_000_000_000;dt=50_000_000;speed=5.
    route=args.out/'synthetic_line.csv';end=helper.line_point(500.)
    route.write_text('s,x,y,z\n0,0,0,0\n500,'+','.join(map(repr,end))+'\n')
    settings=helper.parameters(calibration,route.resolve(),'late_recovery')
    settings.update(initial_gnss_seconds=3.,sparse_gnss_correction=True,
                    gnss_xy_residual_correction=True,body_velocity_output=False,
                    publish_relative_position=False,speed_scale=1.)
    command=helper.command(settings,'late_recovery')
    samples={'velocity':[],'position':[],'diagnostics':[]};assertions=[]
    report={'status':'NOT_COMPLETED','synthetic_only':True,'reference_topic_used':False,
            'input_schedule':'5m/s,20Hz,8seconds; no GNSS first4s; paired goodfixes4.0 through6.9s at10Hz',
            'command':command,'assertions':assertions,
            'source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'helper_sha256':hashlib.sha256(args.helpers.read_bytes()).hexdigest(),
            'calibration_sha256':hashlib.sha256(args.calibration.read_bytes()).hexdigest()}
    process=None;log=None;node=None;started=time.monotonic()
    def check(name,passed,evidence):assertions.append({'name':name,'pass':bool(passed),'evidence':evidence})
    try:
        rclpy.init();node=Node('late_pair_recovery_fixture')
        def spin(deadline):
            while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=min(.003,max(0.,deadline-time.monotonic())))
        clock=node.create_publisher(Clock,'/clock',10)
        wheels=[node.create_publisher(VelocitySensor,'/compat/input/'+name,10) for name in ['front','rear']]
        controller=node.create_publisher(DriverControllerCommand,'/compat/input/command',10)
        fixes={side:node.create_publisher(NavSatFix,'/compat/late_recovery/'+side,10) for side in ['master','rover']}
        node.create_subscription(VelocitySensor,'/compat/late_recovery/velocity',lambda m:samples['velocity'].append({'stamp':helper.stamp_ns(m.header.stamp),'value':m.velocity,'frame':m.header.frame_id}),200)
        def position(m):
            p=m.pose.pose.position
            samples['position'].append({'stamp':helper.stamp_ns(m.header.stamp),'xyz':[p.x,p.y,p.z],'frame':m.header.frame_id,'child':m.child_frame_id})
        node.create_subscription(Odometry,'/compat/late_recovery/position',position,200)
        node.create_subscription(Float64MultiArray,'/compat/late_recovery/diagnostics',lambda m:samples['diagnostics'].append(list(m.data)),100)
        log=(args.out/'estimator_node.log').open('w')
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,preexec_fn=helper.child_setup)
        deadline=time.monotonic()+args.discovery_timeout
        publishers=wheels+[controller,*fixes.values(),clock]
        while time.monotonic()<deadline:
            spin(time.monotonic()+.02)
            if all(p.get_subscription_count()>=1 for p in publishers):break
            if process.poll() is not None:raise RuntimeError('Node exited during discovery')
        ready=all(p.get_subscription_count()>=1 for p in publishers)
        check('DDS_discovery',ready,[p.get_subscription_count() for p in publishers])
        if not ready:raise RuntimeError('DDS discovery deadline')
        wall=time.monotonic()
        for index in range(160):
            spin(wall+index*.05);ns=epoch+index*dt
            clk=Clock();helper.set_stamp(clk.clock,ns);clock.publish(clk);spin(time.monotonic()+.006)
            for publisher in wheels:
                msg=VelocitySensor();helper.set_stamp(msg.header.stamp,ns);msg.header.frame_id='base_link';msg.velocity=speed*3.6;publisher.publish(msg)
            if 80<=index<=138 and index%2==0:
                for side in ['master','rover']:
                    msg=NavSatFix();helper.set_stamp(msg.header.stamp,ns);msg.header.frame_id=side;msg.status.status=0
                    lever=helper.MASTER if side=='master' else helper.ROVER
                    msg.latitude,msg.longitude,msg.altitude=helper.enu_to_lla(helper.antenna_point('line',index*.05,lever),origin)
                    fixes[side].publish(msg)
            spin(time.monotonic()+.006)
            msg=DriverControllerCommand();helper.set_stamp(msg.header.stamp,ns);msg.position=0;controller.publish(msg)
            spin(time.monotonic()+.003)
            if process.poll() is not None:raise RuntimeError('Node exited during synthetic input')
        spin(time.monotonic()+.2)
        v=samples['velocity'];positions=samples['position'];diagnostics=samples['diagnostics']
        elapsed=lambda row:(row['stamp']-epoch)*1e-9
        check('velocity_all_epochs_continues',len(v)>=150 and min((elapsed(r) for r in v),default=99)<.2 and max((elapsed(r) for r in v),default=-1)>7.8,{'count':len(v),'first_s':elapsed(v[0]) if v else None,'last_s':elapsed(v[-1]) if v else None})
        check('velocity_exact_5mps_before_and_after',bool(v) and all(math.isfinite(r['value']) and abs(r['value']-speed)<.001 and r['frame']=='base_link' for r in v),max((abs(r['value']-speed) for r in v),default=None))
        check('velocity_strict_monotonic_stamps',all(b['stamp']>a['stamp'] for a,b in zip(v,v[1:])),len(v))
        check('no_absolute_positions_before_first_fix',all(elapsed(r)>=4. for r in positions),{'count':len(positions),'first_s':elapsed(positions[0]) if positions else None})
        check('late_pair_recovers_by_7p2s',bool(positions) and 4.<=elapsed(positions[0])<=7.2,elapsed(positions[0]) if positions else None)
        late=[r for r in positions if 7.2<=elapsed(r)<=7.95]
        check('absolute_recovery_continues_after_fixes_end',len(late)>=10,len(late))
        check('all_positions_map_base_link',bool(positions) and all(r['frame']=='map' and r['child']=='base_link' for r in positions),len(positions))
        check('position_strict_monotonic_stamps',all(b['stamp']>a['stamp'] for a,b in zip(positions,positions[1:])),len(positions))
        errors=[math.dist(r['xyz'],helper.map_point(helper.line_point(50.+speed*elapsed(r)),yaw,translation)) for r in positions]
        check('recovery_pose_uses_current_epoch_not_reset_time',bool(errors) and max(errors)<.05,{'count':len(errors),'max_error_m':max(errors,default=None)})
        distances=[r[3] for r in diagnostics if len(r)>=4]
        check('integrated_distance_never_restarts',len(distances)>=6 and all(b>=a for a,b in zip(distances,distances[1:])),distances)
        # Diagnostics are emitted once per 20 node publications. A dropped final
        # DDS diagnostic can leave the observed last value one second old. The
        # bound allows that cadence while still rejecting a restart at 4s (<=20m).
        check('distance_remains_full_motion_allowing_diagnostic_cadence',bool(distances) and 34.<=distances[-1]<=40.1,distances[-1] if distances else None)
        report['coverage']={key:len(value) for key,value in samples.items()}
        report['status']='PASS' if all(row['pass'] for row in assertions) else 'FAIL'
    except Exception:
        report['status']='ERROR';report['exception']=traceback.format_exc()
    finally:
        if process is not None:helper.stop(process)
        if log is not None:log.close()
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        report['elapsed_wall_s']=time.monotonic()-started
        (args.out/'late_pair_recovery.json').write_text(json.dumps(helper.json_safe(report),indent=2,allow_nan=False))
        (args.out/'samples.json').write_text(json.dumps(helper.json_safe(samples),indent=2,allow_nan=False))
    print(json.dumps({'status':report['status'],'out':str(args.out),'elapsed_wall_s':report['elapsed_wall_s']}))
    return 0 if report['status']=='PASS' else 1

def main():
    root=Path(__file__).resolve().parents[1]
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True)
    p.add_argument('--helpers',type=Path,default=Path(__file__).with_name('ros_checker_compat.py'))
    p.add_argument('--calibration',type=Path,default=root/'config/map_calibration.json')
    p.add_argument('--domain',type=int,default=53);p.add_argument('--discovery-timeout',type=float,default=3.)
    args=p.parse_args();raise SystemExit(run(args))
if __name__=='__main__':main()
