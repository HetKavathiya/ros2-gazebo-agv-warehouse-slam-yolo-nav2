"""Start the YOLO detector with the virtual-environment Python (no activation needed).

Works from several places (it searches for yolo_detector.py and models/best.pt in this order):
  1. next to this file                      (GitHub repo layout: agv_yolo/)
  2. one folder up: ../scripts, ../models   (source tree of the ROS package)
  3. the installed package share folder     (ros2 launch agv_chassis_description yolo_detector.launch.py)
  4. ~/carparking_ws/agv_yolo_model/        (original development layout)

  ros2 launch agv_chassis_description yolo_detector.launch.py
  ros2 launch <repo>/agv_yolo/yolo_detector.launch.py conf:=0.3 gpu_id:=-1 disable_cudnn:=true
ROS must be sourced in the terminal (it is inherited by the detector).
"""
import os

PACKAGE = 'agv_chassis_description'


def _share_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(PACKAGE)
    except Exception:
        return None


def candidates(here, home, share):
    """Return (script_candidates, model_candidates) in search order."""
    script = [os.path.join(here, 'yolo_detector.py'),
              os.path.join(here, '..', 'scripts', 'yolo_detector.py')]
    model = [os.path.join(here, 'models', 'best.pt'),
             os.path.join(here, '..', 'models', 'best.pt')]
    if share:
        script.append(os.path.join(share, 'scripts', 'yolo_detector.py'))
        model.append(os.path.join(share, 'models', 'best.pt'))
    script.append(os.path.join(home, 'carparking_ws', 'agv_yolo_model', 'yolo_detector.py'))
    model.append(os.path.join(home, 'carparking_ws', 'agv_yolo_model', 'export', 'best.pt'))
    return [os.path.normpath(p) for p in script], [os.path.normpath(p) for p in model]


def first_existing(paths):
    for p in paths:
        if os.path.isfile(p):
            return p
    return None


def generate_launch_description():
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, ExecuteProcess
    from launch.substitutions import LaunchConfiguration

    here = os.path.dirname(os.path.realpath(__file__))
    home = os.path.expanduser('~')
    script_c, model_c = candidates(here, home, _share_dir())

    script = first_existing(script_c)
    if script is None:
        raise FileNotFoundError('yolo_detector.py not found. Looked in:\n  ' + '\n  '.join(script_c))
    model = first_existing(model_c) or model_c[0]
    print(f'[yolo_detector.launch] script: {script}')
    print(f'[yolo_detector.launch] model : {model}' + ('' if os.path.isfile(model) else '   <-- NOT FOUND'))

    default_python = os.path.join(home, 'yolo_env', 'bin', 'python3')

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=model),
        DeclareLaunchArgument('venv_python', default_value=default_python),
        DeclareLaunchArgument('conf', default_value='0.4'),
        DeclareLaunchArgument('gpu_id', default_value='0'),
        DeclareLaunchArgument('disable_cudnn', default_value='false'),
        ExecuteProcess(
            cmd=[LaunchConfiguration('venv_python'), script, '--ros-args',
                 '-p', 'use_sim_time:=true',
                 '-p', ['model_path:=', LaunchConfiguration('model')],
                 '-p', ['conf:=', LaunchConfiguration('conf')],
                 '-p', ['gpu_id:=', LaunchConfiguration('gpu_id')],
                 '-p', ['disable_cudnn:=', LaunchConfiguration('disable_cudnn')]],
            additional_env={'PYTHONNOUSERSITE': '1'},   # ignore ~/.local packages
            output='screen'),
    ])
