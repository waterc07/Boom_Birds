"""Single-executor hardware dry-run candidate; sensing is supplied externally."""
import argparse
import json
from pathlib import Path
import yaml


def loadCandidate(path):
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data,dict) or data.get("schema") != 1:
        raise ValueError("candidate_schema")
    control,task = data.get("control"),data.get("task")
    if not isinstance(control,dict) or not isinstance(task,dict):
        raise ValueError("candidate_parameters")
    required = {"dry_run":True,"allow_arming":False,"allow_hardware_actions":False,
                "require_session":True,"control_mode":"companion_attitude","backend":"mavros"}
    for name,value in required.items():
        if control.get(name) != value or type(control.get(name)) is not type(value):
            raise ValueError("candidate_control_"+name)
    required = {"dry_run":True,"start":False,"handover_enabled":False,
                "platform_landing_enabled":False,"feature_tracking_enabled":True,
                "control_mode":"companion_attitude"}
    for name,value in required.items():
        if task.get(name) != value or type(task.get(name)) is not type(value):
            raise ValueError("candidate_task_"+name)
    if control.get("platform_config_file") or task.get("sih_pid",0) or control.get("sih_pid",0):
        raise ValueError("candidate_no_platform_or_simulation")
    for params,names in ((control,("attitude_config_file",)),
                         (task,("attitude_config_file","flight_task_file","tag_config_file","obstacle_config_file"))):
        for name in names:
            if not isinstance(params.get(name),str) or not Path(params[name]).is_file():
                raise ValueError("candidate_file_"+name)
    accuracy = data.get("max_accuracy_m",.2)
    if type(accuracy) not in (int,float) or not 0 < accuracy <= .2:
        raise ValueError("candidate_accuracy_limit")
    return data


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config",required=True)
    parser.add_argument("--check-only",action="store_true")
    args=parser.parse_args(argv)
    data=loadCandidate(args.config)
    from boom_birds_control.px4_interface_node import Px4InterfaceNode
    from boom_birds_control.mavros_ekf_reference import MavrosEkfReference
    from .tag_flight_node import TagFlightNode
    from .feature_ekf_bridge import DryRunFeatureEkfBridge
    if args.check_only:
        print(json.dumps(dict(result="CONFIG_AND_IMPORTS_OK",dry_run=True,
            live_control_enabled=False,config=str(Path(args.config).resolve()))))
        return 0
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
    rclpy.init(args=[])
    control=mission=reader=executor=None
    try:
        control=Px4InterfaceNode(parameter_overrides=[Parameter(k,value=v) for k,v in data["control"].items()])
        mission=TagFlightNode(parameter_overrides=[Parameter(k,value=v) for k,v in data["task"].items()])
        control.backend.ekf_reference_required=True
        control.backend._observationStreamsRequested=False
        reader=MavrosEkfReference(control,control.backend)
        bridge=DryRunFeatureEkfBridge(mission,control,reader,max_accuracy_m=data.get("max_accuracy_m",.2))
        pub=control.create_publisher(String,"/boom_birds/feature_ekf_candidate/status",10)
        control.create_timer(.2,lambda:pub.publish(String(data=json.dumps(bridge.report(),allow_nan=False))))
        executor=SingleThreadedExecutor()
        executor.add_node(control)
        executor.add_node(mission)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None: executor.shutdown(timeout_sec=2.)
        if reader is not None: reader.close()
        if control is not None:
            control.shutdown()
            control.destroy_node()
        if mission is not None: mission.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    return 0


if __name__=="__main__":
    raise SystemExit(main())
