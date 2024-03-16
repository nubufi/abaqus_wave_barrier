from pydantic import BaseModel
from typing import List

class SoilLayer(BaseModel):
    layer_name: str
    damping_ratio: float
    elastic_modulus: float # Pa
    poisson_ratio: float
    density: float # kg/m^3
    thickness: float # m
    shear_wave_velocity: float # m/s

class SoilProfile(BaseModel):
    layers: List[SoilLayer]

class TrenchBarrier(BaseModel):
    width: float # m
    length: float # m
    height: float # m
    ditch2source: float # m
    ditch2ditch: float  # m
    number_of_ditch: int

class SheetPileBarrier(BaseModel):
    height: float
    number_of_sheet_pile: int
    elastic_modulus: float # Pa
    density: float # kg/m^3
    poisson_ratio: float

class RubberChipBarrier(BaseModel):
    elastic_modulus: float
    density: float
    damping_ratio: float
    shear_wave_velocity: float

class Motion(BaseModel):
    accelerometer_pattern: List[str] # m
    amplitude: float # m
    frequency: float # Hz
    duration: float # s
    time_step: float # s

class Input(BaseModel):
    mesh_size: float # m
    model_name: str
    dimension: str # 2D, 2D_planar or 3D
    soil_profile: SoilProfile
    barrier: TrenchBarrier | SheetPileBarrier | RubberChipBarrier
    motion: Motion
    width: float # m
    height: float # m
    source_size:str # m
