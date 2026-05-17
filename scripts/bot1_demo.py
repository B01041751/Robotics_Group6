#!/usr/bin/env python3
import os
import subprocess
import time
import rospkg


def _load_spawn_config():
    try:
        pkg_path = rospkg.RosPack().get_path('com760cw2_group6')
        env_file = os.path.join(pkg_path, 'worlds', 'spawn_config.env')
        with open(env_file) as f:
            for line in f:
                line = line.strip()
                if line.startswith('export '):
                    line = line[7:]
                if '=' in line and not line.startswith('#'):
                    key, _, val = line.partition('=')
                    os.environ.setdefault(key.strip(), val.strip())
    except Exception as e:
        print(f'[bot1] WARNING: Could not load spawn_config.env: {e}')


def _delete_existing():
    try:
        result = subprocess.run(
            ['rosservice', 'call', '/gazebo/delete_model', '{model_name: Group6Bot_1}'],
            capture_output=True, text=True, timeout=5.0
        )
        if 'success: True' in result.stdout:
            print('[bot1] Removed previous Bot 1 from Gazebo')
        time.sleep(0.5)
    except Exception:
        pass


def main():
    _load_spawn_config()

    goal_x = os.environ.get('GAS1_X', '?')
    goal_y = os.environ.get('GAS1_Y', '?')

    print('\n' + '=' * 60)
    print('  DEMO: Bot 1 — Bug2 navigation')
    print(f'  Spawn : ({os.environ.get("BOT1_X","?")} , {os.environ.get("BOT1_Y","?")})')
    print(f'  Goal  : Gas Zone 1 ({goal_x}, {goal_y})')
    print('=' * 60 + '\n')

    _delete_existing()

    pkg_path    = rospkg.RosPack().get_path('com760cw2_group6')
    launch_file = os.path.join(pkg_path, 'launch', 'bot1_demo.launch')

    proc = subprocess.Popen(['roslaunch', launch_file])

    print('[bot1] Running... (Ctrl-C to stop)\n')

    try:
        proc.wait()
    except KeyboardInterrupt:
        print('\n[bot1] Stopped')
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()

    print('[bot1] Done\n')


if __name__ == '__main__':
    main()
