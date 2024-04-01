import json
from model import *
from pandas import read_excel
import os


def read_json(file_name) -> Input:
    with open(file_name) as f:
        data = json.load(f)

    return Input(**data)


def string_to_list(string: str):
    # example string:
    return [float(i) for i in string[1:-1].split(",")]


def create_dir(target_folder: str):
    if not os.path.exists(target_folder):
        os.mkdir(target_folder)


def excel_to_json(file_name: str, target_folder: str):
    soil_parameters = read_excel(file_name, sheet_name="SoilProfile").values
    model_parameters = (
        read_excel(file_name, sheet_name="Model", header=None).iloc[:, 1].values
    )
    barrier_parameters = read_excel(file_name, sheet_name="Barriers").values
    motion_parameters = (
        read_excel(file_name, sheet_name="Motion", header=None).iloc[:, 1].values
    )

    input_dict = {
        "model_name": model_parameters[0],
        "width": model_parameters[1],
        "height": model_parameters[2],
        "mesh_size": model_parameters[3],
        "dimension": model_parameters[4],
        "soil_profile": {
            "layers": [
                {
                    "layer_name": layer[0],
                    "elastic_modulus": layer[1],
                    "poisson_ratio": layer[2],
                    "density": layer[3],
                    "thickness": layer[4],
                    "shear_wave_velocity": layer[5],
                    "damping_ratio": layer[6],
                }
                for layer in soil_parameters
            ]
        },
        "barrier": [
            {
                "barrier_name": barrier[0],
                "length": barrier[1],
                "width": barrier[2],
                "height": barrier[3],
                "distance_to_source": barrier[4],
                "elastic_modulus": barrier[5],
                "density": barrier[6],
                "poisson_ratio": barrier[7],
                "damping_ratio": barrier[8],
                "shear_wave_velocity": barrier[9],
            }
            for barrier in barrier_parameters
        ],
        "motion": {
            "accelerometer_pattern": string_to_list(motion_parameters[0]),
            "amplitude": motion_parameters[1],
            "frequency": motion_parameters[2],
            "time_step": motion_parameters[3],
            "duration": motion_parameters[4],
            "source_size": motion_parameters[5],
        },
    }

    create_dir(target_folder)

    with open(f"{target_folder}/input.json", "w") as f:
        json.dump(input_dict, f, indent=4)
