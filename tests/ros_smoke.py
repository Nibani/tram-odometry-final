"""Actual ROS/DDS black-box smoke test, run after sourcing a Humble workspace."""
import argparse,json,math,os,resource,subprocess,tempfile,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from tram_vehicle_msgs.msg import VelocitySensor,DriverControllerCommand
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_prefix
from rosgraph_msgs.msg import Clock

def executable():
    return str(Path(get_package_prefix('reserve_odometry'))/'lib/reserve_odometry/reserve_odometry_node')

def limit_cores():
    os.sched_setaffinity(0,set(sorted(os.sched_getaffinity(0))[:2]))

def rss_kib(pid):
    try:
        for line in Path(f'/proc/{pid}/status').read_text().splitlines():
            if line.startswith('VmHWM:'):return int(line.split()[1])
    except FileNotFoundError:pass
    return 0

def ecef(lat,lon,h):
    lat=math.radians(lat);lon=math.radians(lon);e2=6.6943799901413165e-3;n=6378137/math.sqrt(1-e2*math.sin(lat)**2)
    return [(n+h)*math.cos(lat)*math.cos(lon),(n+h)*math.cos(lat)*math.sin(lon),(n*(1-e2)+h)*math.sin(lat)]

def lla(enu):
    lat=math.radians(55.805);lon=math.radians(37.425);origin=ecef(55.805,37.425,170.)
    rows=[[-math.sin(lon),math.cos(lon),0],[-math.sin(lat)*math.cos(lon),-math.sin(lat)*math.sin(lon),math.cos(lat)],[math.cos(lat)*math.cos(lon),math.cos(lat)*math.sin(lon),math.sin(lat)]]
    x,y,z=[origin[j]+sum(enu[i]*rows[i][j] for i in range(3)) for j in range(3)];rho=math.hypot(x,y);la=math.atan2(z,rho*(1-6.6943799901413165e-3))
    for _ in range(10):
        n=6378137/math.sqrt(1-6.6943799901413165e-3*math.sin(la)**2);h=rho/math.cos(la)-n;la=math.atan2(z,rho*(1-6.6943799901413165e-3*n/(n+h)))
    return math.degrees(la),math.degrees(math.atan2(y,x)),h

def stamp_ns(stamp):return stamp.sec*1000000000+stamp.nanosec

def run_case(mode,output):
    os.environ['ROS_DOMAIN_ID']={'missing':'41','mapped':'42','hypotheses':'45'}[mode]
    mapped=mode!='missing';target_y=5. if mode=='hypotheses' else 0.
    rclpy.init();node=Node('reserve_odometry_test_'+mode)
    fp=node.create_publisher(VelocitySensor,'/vehicle/front_bogie_velocity',10);rp=node.create_publisher(VelocitySensor,'/vehicle/rear_bogie_velocity',10);cp=node.create_publisher(DriverControllerCommand,'/vehicle/driver_position_cmd',10)
    mp=node.create_publisher(NavSatFix,'/sensing/gnss/master/fix',10);gp=node.create_publisher(NavSatFix,'/sensing/gnss/rover/fix',10)
    velocities=[];poses=[]
    node.create_subscription(VelocitySensor,'/result/velocity',lambda m:velocities.append((stamp_ns(m.header.stamp),m.velocity,time.time_ns())),100)
    node.create_subscription(Odometry,'/result/position',lambda m:poses.append((stamp_ns(m.header.stamp),m.pose.pose.position.x,m.header.frame_id,m.pose.pose.position.y)),100)
    with tempfile.TemporaryDirectory() as tmp:
        route=Path(tmp)/'route.csv';route.write_text('s,x,y,z\n0,0,0,0\n500,500,0,0\n')
        cmd=[executable(),'--ros-args','-p','use_sim_time:=false']
        if mapped:
            alternate=Path(tmp)/'alternate.csv';alternate.write_text('s,x,y,z\n0,0,5,0\n500,500,5,0\n')
            paths=[str(route),str(alternate)] if mode=='hypotheses' else [str(route)]
            cmd+=['-p','route_files:='+json.dumps(paths),'-p','closed_route:=false']
        with (output/(mode+'_node.log')).open('w') as log:
            before=resource.getrusage(resource.RUSAGE_CHILDREN);begin=time.monotonic();peak_rss=0
            process=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,preexec_fn=limit_cores)
            try:
                until=time.monotonic()+2.
                while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
                start=time.time_ns();wall=time.monotonic();i=0
                while time.monotonic()-wall<8.:
                    target=wall+i*.05
                    while time.monotonic()<target:rclpy.spin_once(node,timeout_sec=.002)
                    now=time.time_ns();elapsed=(now-start)*1e-9;stamp=node.get_clock().now().to_msg()
                    for pub in [fp,rp]:
                        msg=VelocitySensor();msg.header.stamp=stamp;msg.header.frame_id='base_link';msg.velocity=18.;pub.publish(msg)
                    if mapped and i%2==0:
                        shift=100. if elapsed>3.5 and mode=='mapped' else 0.
                        fix_y=0. if elapsed>3.5 and mode=='hypotheses' else target_y
                        for pub,x in [(mp,-9.873),(gp,2.563)]:
                            msg=NavSatFix();msg.header.stamp=stamp;msg.status.status=0;msg.latitude,msg.longitude,msg.altitude=lla([50.+5.*elapsed+x+shift,fix_y,3.]);pub.publish(msg)
                    # No controller at all in missing mode; a full one-second gap
                    # in mapped mode exercises publication independent of commands.
                    if mapped and not 4.<elapsed<5.:
                        msg=DriverControllerCommand();msg.header.stamp=stamp;msg.position=0;cp.publish(msg)
                    rclpy.spin_once(node,timeout_sec=.003);i+=1;peak_rss=max(peak_rss,rss_kib(process.pid))
                until=time.monotonic()+.3
                while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
                assert process.poll() is None,'ROS node crashed'
            finally:
                process.terminate()
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:process.kill();process.wait()
    after=resource.getrusage(resource.RUSAGE_CHILDREN);cpu_cores=(after.ru_utime+after.ru_stime-before.ru_utime-before.ru_stime)/(time.monotonic()-begin)
    (output/(mode+'_samples.json')).write_text(json.dumps({'velocity':velocities,'poses':poses}))
    assert 0<peak_rss<512*1024,('memory ceiling',peak_rss)
    assert cpu_cores<2.01,('CPU ceiling',cpu_cores)
    assert len(velocities)>=100,(mode,'output count',len(velocities))
    valid=[v for v in velocities if v[0]>start+1_000_000_000 and v[0]<start+7_900_000_000]
    assert all(math.isfinite(v[1]) and abs(v[1]-5.)<.1 for v in valid),'wrong velocity'
    gaps=[(b[0]-a[0])*1e-9 for a,b in zip(valid,valid[1:])];assert min(gaps)>0,'nonmonotonic stamps';assert max(gaps)<.25,'publication blackout'
    latency=sorted((v[2]-v[0])*1e-6 for v in valid);assert latency[int(.99*(len(latency)-1))]<100.,'p99 DDS latency >100ms'
    assert max(latency)<250.,'peak DDS latency >250ms'
    if mapped:
        selected=[p for p in poses if p[0]>start+3_200_000_000 and p[0]<start+7_900_000_000]
        assert selected and all(p[2]=='map_enu' for p in selected),'initialization missing'
        error=max(math.hypot(p[1]-(50+5*(p[0]-start)*1e-9),p[3]-target_y) for p in selected);assert error<.3,('position/late GNSS error',error)
    else:error=None
    report={'mode':mode,'outputs':len(velocities),'mean_hz':(len(valid)-1)/((valid[-1][0]-valid[0][0])*1e-9),'max_gap_s':max(gaps),'latency_p99_ms':latency[int(.99*(len(latency)-1))],'latency_max_ms':max(latency),'position_max_error_m':error,'node_peak_rss_kib':peak_rss,'mean_cpu_cores':cpu_cores,'affinity_cores':2,'status':'PASS'}
    node.destroy_node();rclpy.shutdown();return report

def simulated_clock_case(output):
    os.environ['ROS_DOMAIN_ID']='43';rclpy.init();node=Node('sim_clock_test')
    clockpub=node.create_publisher(Clock,'/clock',10)
    wheels=[node.create_publisher(VelocitySensor,p,10) for p in ['/vehicle/front_bogie_velocity','/vehicle/rear_bogie_velocity']]
    values=[];node.create_subscription(VelocitySensor,'/result/velocity',lambda m:values.append((stamp_ns(m.header.stamp),m.velocity,time.monotonic())),100)
    with (output/'sim_clock_node.log').open('w') as log:
        process=subprocess.Popen([executable(),'--ros-args','-p','use_sim_time:=true'],stdout=log,stderr=subprocess.STDOUT,preexec_fn=limit_cores)
        try:
            until=time.monotonic()+2.
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
            boundaries=[]
            for phase,start in enumerate([100.,102.,1.]):
                # Phase1 rolls the clock backward only0.5s; phase2 imitates a new bag.
                boundaries.append((len(values),time.monotonic()))
                for i in range(51):
                    ns=int((start+i*.05)*1e9);clk=Clock();clk.clock.sec=ns//10**9;clk.clock.nanosec=ns%10**9;clockpub.publish(clk)
                    until=time.monotonic()+.012
                    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.001)
                    for pub in wheels:
                        msg=VelocitySensor();msg.header.stamp=clk.clock;msg.velocity=18.;pub.publish(msg)
                    until=time.monotonic()+.038
                    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.001)
            assert process.poll() is None,'sim clock node crashed'
        finally:
            process.terminate();process.wait(timeout=5)
    counts=[]
    for i,(first,wall) in enumerate(boundaries):
        end=boundaries[i+1][0] if i+1<len(boundaries) else len(values);v=values[first:end];counts.append(len(v))
        assert len(v)>30,('clock recovery missing',i,len(v))
        assert v[0][2]-wall<.25,('clock recovery >250ms',i,v[0][2]-wall)
        assert all(abs(x[1]-5.)<.1 for x in v),('clock velocity',i)
        assert all(b[0]>a[0] for a,b in zip(v,v[1:])),('phase stamps',i)
    (output/'sim_clock_samples.json').write_text(json.dumps(values));node.destroy_node();rclpy.shutdown()
    return {'mode':'simulated_clock','phase_output_counts':counts,'backward_jumps_s':[.5,103.5],'status':'PASS'}

def bag_replay_case(output):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    os.environ['ROS_DOMAIN_ID']='44';rclpy.init();node=Node('actual_bag_test');values=[]
    node.create_subscription(VelocitySensor,'/result/velocity',lambda m:values.append((stamp_ns(m.header.stamp),m.velocity)),100)
    with tempfile.TemporaryDirectory() as tmp:
        bag=str(Path(tmp)/'input');writer=rosbag2_py.SequentialWriter();writer.open(rosbag2_py.StorageOptions(uri=bag,storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
        topics=['/vehicle/front_bogie_velocity','/vehicle/rear_bogie_velocity','/vehicle/driver_position_cmd']
        for i,topic in enumerate(topics):writer.create_topic(rosbag2_py.TopicMetadata(name=topic,type='tram_vehicle_msgs/msg/'+('DriverControllerCommand' if i==2 else 'VelocitySensor'),serialization_format='cdr'))
        for i in range(121):
            ns=100_000_000_000+i*50_000_000
            for j,topic in enumerate(topics):
                msg=DriverControllerCommand() if j==2 else VelocitySensor();msg.header.stamp.sec=ns//10**9;msg.header.stamp.nanosec=ns%10**9
                if j<2:msg.velocity=18.
                else:msg.position=0
                writer.write(topic,serialize_message(msg),ns+j)
        del writer
        with (output/'bag_node.log').open('w') as log,(output/'bag_player.log').open('w') as playlog:
            process=subprocess.Popen([executable(),'--ros-args','-p','use_sim_time:=true'],stdout=log,stderr=subprocess.STDOUT,preexec_fn=limit_cores);player=None
            try:
                until=time.monotonic()+2.
                while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
                player=subprocess.Popen(['ros2','bag','play',bag,'--clock','100','--rate','2'],stdout=playlog,stderr=subprocess.STDOUT)
                deadline=time.monotonic()+15.
                while player.poll() is None and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.005)
                assert player.poll()==0,'ros2 bag play failed/timed out'
                until=time.monotonic()+.3
                while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.005)
            finally:
                if player is not None and player.poll() is None:player.terminate();player.wait(timeout=5)
                process.terminate();process.wait(timeout=5)
    assert len(values)>100,('bag output count',len(values));assert all(abs(v-5.)<.1 for _,v in values),'bag velocity'
    assert all(b[0]>a[0] for a,b in zip(values,values[1:])),'bag timestamps';assert max((b[0]-a[0])*1e-9 for a,b in zip(values,values[1:]))<.25,'bag publication gap'
    node.destroy_node();rclpy.shutdown();return {'mode':'actual_sqlite_rosbag_clock','outputs':len(values),'rate':2,'status':'PASS'}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    reports=[run_case(m,a.out) for m in ['missing','mapped','hypotheses']];reports.append(simulated_clock_case(a.out));reports.append(bag_replay_case(a.out));report={'ros':'Humble','cases':reports,'child_max_rss_kib':resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss}
    (a.out/'ros_smoke.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
