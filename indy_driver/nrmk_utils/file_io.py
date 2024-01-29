import os
import shutil
from shutil import disk_usage
import numpy as np
import json
import yaml
import pickle


class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


def save_json(filepath, data):
    with open(filepath, 'w', encoding="utf-8") as outfile:
        json.dump(data, outfile, ensure_ascii=False, indent=2, cls=NumpyEncoder)


def load_json(filepath):
    with open(filepath, 'r', encoding="utf-8") as json_file:
        data = json.load(json_file)
        return data


def load_yaml(filepath):
    with open(filepath, 'r') as stream:
        line = stream.readline()
        if 'YAML:1.0' in line:
            _ = stream.readline()
            yml_dict = yaml.safe_load(stream)  # yaml.load(f) call has been deprecated in the 5.x line
            save_yaml(filepath, yml_dict)
        else:
            stream.seek(0)
            yml_dict = yaml.safe_load(stream)
    return yml_dict


def save_yaml(filepath, ymlDict):
    with open(filepath, 'w') as yaml_file:
        config_to_write = {}
        for key, val in ymlDict.items():
            config_to_write[key] = val
        yaml.dump(config_to_write, yaml_file, default_flow_style=False)
    return True


def load_text(filepath):
    with open(filepath, 'r', encoding="utf-8") as file:
        data = file.read()
        return data


def save_text(filepath, data):
    with open(filepath, 'w', encoding="utf-8") as file:
        return file.write(data)


def save_pickle(filepath, obj):
    with open(filepath, 'wb') as f:
        pickle.dump(obj, f)


def load_pickle(filepath):
    with open(filepath, 'rb') as f:
        obj = pickle.load(f)
    return obj


def pickleable(obj):
    try:
        pickle.dumps(obj)
    except Exception as e:
        return e
    return True


def create_dir(dir):
    os.makedirs(dir, exist_ok=True)


def delete_dir(dir):
    if not os.path.isdir(dir): return
    shutil.rmtree(dir)


def delete_file(path):
    #     if not os.path.isdir(path): return
    if os.path.isfile(path):
        os.remove(path)
    else:
        shutil.rmtree(path, ignore_errors=True)


def get_memory_usage_gb():
    total, used, available = [val / 1024 / 1024 / 1024 for val in disk_usage('/')]
    return total, used, available


def get_pkg_path():
    dir_seq = os.path.dirname(__file__).split(os.sep)
    i_pkg = dir_seq.index("pkg")
    pkg_path = os.sep.join(dir_seq[:i_pkg+1])
    return pkg_path


def get_proj_path():
    pkg_path =get_pkg_path()
    return os.path.dirname(pkg_path)


def get_home_path():
    dir_seq = os.path.dirname(__file__).split(os.sep)
    if dir_seq[1] != 'home':
        raise(RuntimeError("Get home is only supported in the sub-home directory."
                           + " Please place file_io.py file under user home"))
    return os.path.join(os.sep, *dir_seq[:3])
