import json
from model import *


def read_json(file_name):
    print("----------------")
    with open(file_name) as f:
        data = json.load(f)

    soil_profile_data = data.get("soil_profile", {})
    layers = [
        SoilLayer(
            layer_name=layer["layer_name"],
            damping_ratio=layer["damping_ratio"],
            elastic_modulus=layer["elastic_modulus"],
            poisson_ratio=layer["poisson_ratio"],
            density=layer["density"],
            thickness=layer["thickness"],
            shear_wave_velocity=layer["shear_wave_velocity"],
        )
        for layer in soil_profile_data.get("layers", [])
    ]
    soil_profile = SoilProfile(layers=layers)

    barriers_data = data.get("barrier", [])

    barriers = [
        Barrier(
            barrier_name=barrier["barrier_name"],
            width=barrier["width"],
            length=barrier["length"],
            height=barrier["height"],
            distance_to_source=barrier["distance_to_source"],
            elastic_modulus=barrier["elastic_modulus"],
            density=barrier["density"],
            poisson_ratio=barrier["poisson_ratio"],
            shear_wave_velocity=barrier["shear_wave_velocity"],
            damping_ratio=barrier["damping_ratio"],
        )
        for barrier in barriers_data
    ]

    motion_data = data.get("motion", {})
    motion = Motion(
        accelerometer_pattern=motion_data["accelerometer_pattern"],
        amplitude=motion_data.get("amplitude", 0),
        frequency=motion_data.get("frequency", 0),
        duration=motion_data.get("duration", 0),
        time_step=motion_data.get("time_step", 0),
        source_size=motion_data.get("source_size", ""),
    )

    input_instance = Input(
        mesh_size=data.get("mesh_size", 0),
        model_name=data.get("model_name", ""),
        dimension=data.get("dimension", ""),
        soil_profile=soil_profile,
        barrier=barriers,
        motion=motion,
        width=data.get("width", 0),
        height=data.get("height", 0),
    )

    return input_instance
