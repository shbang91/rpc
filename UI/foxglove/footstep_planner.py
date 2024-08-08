import os
import time
import yaml
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import glob
import numpy as np

# Define the directory to watch
WATCHED_DIR = 'experiment_data'
# variables loaded from yaml
t_ini_footsteps_planned, t_end_footsteps_planned = [], []
rfoot_contact_pos, rfoot_contact_ori = [], []
lfoot_contact_pos, lfoot_contact_ori = [], []
step_num = 0

def remove_yaml_files(directory):
    if not os.path.isdir(directory):
        raise ValueError(f"The directory {directory} does not exist")
    # Use glob to find all .yaml files in the directory
    yaml_files = glob.glob(os.path.join(directory, '*.yaml'))
    # Remove each .yaml file found
    print("***  removing old experiment data  ***")
    for file_path in yaml_files:
        try:
            os.remove(file_path)
            print(f"Removed file: {file_path}")
        except OSError as e:
            print(f"Error removing file {file_path}: {e}")
    print("***  old experiment data removed  ***")

class yamlHandler(FileSystemEventHandler):
    def on_created(self, event):
        # Check if the created file is a YAML file
        if event.is_directory or not event.src_path.endswith('.yaml'):
            return
        print(f"New YAML file detected: {event.src_path}")
        self.process_file(event.src_path)

    def process_file(self, file_path):
        with open(file_path, "r") as stream:
            try:
                time.sleep(0.01)    #wait for yaml to be generated  -- might want to replace with a function to check when it's actually been written
                cfg = yaml.load(stream, Loader=yaml.FullLoader)
                t_ini_footsteps_planned.append(np.array(cfg["temporal_parameters"]["initial_time"]))
                t_end_footsteps_planned.append(np.array(cfg["temporal_parameters"]["final_time"]))
                rfoot_contact_pos.append(np.array(cfg["contact"]["right_foot"]["pos"]))
                rfoot_contact_ori.append(np.array(cfg["contact"]["right_foot"]["ori"]))
                lfoot_contact_pos.append(np.array(cfg["contact"]["left_foot"]["pos"]))
                lfoot_contact_ori.append(np.array(cfg["contact"]["left_foot"]["ori"]))
                res = ''.join(filter(lambda i: i.isdigit(), str(file_path)))
                global step_num
                step_num = int(res)
            except yaml.YAMLError as exc:
                print(exc)

def main():
    remove_yaml_files(WATCHED_DIR)
    event_handler = yamlHandler()
    observer = Observer()
    observer.schedule(event_handler, path=WATCHED_DIR, recursive=False)
    observer.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
    observer.join()

if __name__ == "__main__":
    main()