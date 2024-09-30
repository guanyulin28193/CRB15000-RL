import subprocess
import time
import threading
import os

RESTART_INTERVAL = 1800  # 1小时（秒）
BUILD_EXE_PATH = r".\Builds\BT-VF\ABB-RL.exe"
BUILD_ARGS = ["-batchmode", "-nographics"]
NUM_INSTANCES = 6  # 你想要运行的实例数量

os.chdir(r"C:\Users\18125\CRB15000-RL")

def run_single_build(instance_id):
    while True:
        start_time = time.time()
        
        print(f"实例 {instance_id} 开始运行构建程序，时间：{time.strftime('%Y-%m-%d %H:%M:%S')}")
        process = subprocess.Popen([BUILD_EXE_PATH] + BUILD_ARGS, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
        
        try:
            while True:
                output = process.stdout.readline()
                if output:
                    print(f"实例 {instance_id}: {output.strip()}")
                
                if process.poll() is not None:
                    break
                
                if time.time() - start_time >= RESTART_INTERVAL:
                    print(f"实例 {instance_id} 1小时后重启构建程序，时间：{time.strftime('%Y-%m-%d %H:%M:%S')}")
                    process.terminate()
                    time.sleep(5)  # 等待进程终止
                    break
        
        except KeyboardInterrupt:
            print(f"实例 {instance_id} 用户中断进程。正在终止...")
            process.terminate()
            break

def run_multiple_builds():
    threads = []
    for i in range(NUM_INSTANCES):
        thread = threading.Thread(target=run_single_build, args=(i+1,))
        threads.append(thread)
        thread.start()
        time.sleep(1)
    
    for thread in threads:
        thread.join()

if __name__ == "__main__":
    run_multiple_builds()