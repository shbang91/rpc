import os
import time
import yaml
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import glob
import numpy as np

# Define the directory to watch
WATCHED_DIR = "experiment_data"
# variables loaded from yaml
t_ini_footsteps_planned, t_end_footsteps_planned = [], []
rfoot_contact_pos, rfoot_contact_ori = [], []
lfoot_contact_pos, lfoot_contact_ori = [], []


class StepData:
    def __init__(
        self, yaml_num, rf_steps_taken, rf_steps_total, lf_steps_taken, lf_steps_total
    ):
        self.yaml_num = yaml_num
        self.rf_steps_taken = rf_steps_taken
        self.rf_steps_total = rf_steps_total
        self.lf_steps_taken = lf_steps_taken
        self.lf_steps_total = lf_steps_total

    def step_update(self, res, r_len, l_len):
        self.yaml_num = res
        self.rf_steps_taken = self.rf_steps_total
        self.lf_steps_taken = self.lf_steps_total
        self.rf_steps_total += r_len
        self.lf_steps_total += l_len

    def steps_to_update(self):
        # Returns dict of projections mapped to the yaml index
        feet = {}
        for i in range(self.rf_steps_taken, self.rf_steps_total):
            rf = "proj_rf" + str(i)
            feet[rf] = i - self.rf_steps_taken
        for i in range(self.lf_steps_taken, self.lf_steps_total):
            lf = "proj_lf" + str(i)
            feet[lf] = i - self.lf_steps_taken
        return feet


sd = StepData(0, 0, 0, 0, 0)


def yaml_check(file_path, search_line):
    try:
        with open(file_path, "r") as file:
            lines = file.readlines()
        # Check if the search_line exists in the list of lines
        for line in lines:
            if search_line in line:
                return True
        return False
    except FileNotFoundError:
        print(f"File not found: {file_path}")
        return False
    except Exception as e:
        print(f"An error occurred: {e}")
        return False


def remove_yaml_files(directory):
    if not os.path.isdir(directory):
        raise ValueError(f"The directory {directory} does not exist")
    # Use glob to find all .yaml files in the directory
    yaml_files = glob.glob(os.path.join(directory, "*.yaml"))
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
        if event.is_directory or not event.src_path.endswith(".yaml"):
            return
        print(f"New YAML file detected: {event.src_path}")
        self.process_file(event.src_path)

    def process_file(self, file_path):
        with open(file_path, "r") as stream:
            try:
                while not yaml_check(
                    file_path, "  vrp:"
                ):  # Wait until yaml data is generated
                    time.sleep(0.01)
                cfg = yaml.load(stream, Loader=yaml.FullLoader)
                t_ini_footsteps_planned.append(
                    np.array(cfg["temporal_parameters"]["initial_time"])
                )
                t_end_footsteps_planned.append(
                    np.array(cfg["temporal_parameters"]["final_time"])
                )
                rfoot_contact_pos.append(np.array(cfg["contact"]["right_foot"]["pos"]))
                rfoot_contact_ori.append(np.array(cfg["contact"]["right_foot"]["ori"]))
                lfoot_contact_pos.append(np.array(cfg["contact"]["left_foot"]["pos"]))
                lfoot_contact_ori.append(np.array(cfg["contact"]["left_foot"]["ori"]))
                res = "".join(filter(lambda i: i.isdigit(), str(file_path)))
                sd.step_update(
                    int(res),
                    len(rfoot_contact_pos[int(res)]),
                    len(lfoot_contact_pos[int(res)]),
                )
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
