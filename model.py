class SoilLayer(object):
    def __init__(
        self,
        layer_name,
        damping_ratio,
        elastic_modulus,
        poisson_ratio,
        density,
        thickness,
        shear_wave_velocity,
    ):
        self.layer_name = layer_name
        self.damping_ratio = damping_ratio
        self.elastic_modulus = elastic_modulus  # Pa
        self.poisson_ratio = poisson_ratio
        self.density = density  # kg/m^3
        self.thickness = thickness  # m
        self.shear_wave_velocity = shear_wave_velocity  # m/s


class SoilProfile(object):
    def __init__(self, layers):
        self.layers = layers  # List[SoilLayer]


class Barrier(object):
    def __init__(
        self,
        barrier_name,
        width,
        length,
        height,
        distance_to_source,
        elastic_modulus,
        density,
        poisson_ratio,
        shear_wave_velocity,
        damping_ratio,
    ):
        self.barrier_name = barrier_name
        self.width = width  # m
        self.length = length  # m
        self.height = height  # m
        self.distance_to_source = distance_to_source  # m
        self.elastic_modulus = elastic_modulus  # Pa
        self.density = density  # kg/m^3
        self.poisson_ratio = poisson_ratio
        self.shear_wave_velocity = shear_wave_velocity
        self.damping_ratio = damping_ratio


class Motion(object):
    def __init__(
        self,
        accelerometer_pattern,
        amplitude,
        frequency,
        duration,
        time_step,
        source_size,
    ):
        self.accelerometer_pattern = accelerometer_pattern  # List[str]  # m
        self.amplitude = amplitude  # m
        self.frequency = frequency  # Hz
        self.duration = duration  # s
        self.time_step = time_step  # s
        self.source_size = source_size  # m


class Input(object):
    def __init__(
        self,
        mesh_size,
        model_name,
        dimension,
        soil_profile,
        barrier,
        motion,
        width,
        height,
    ):
        self.mesh_size = mesh_size  # m
        self.model_name = model_name
        self.dimension = dimension  # 2D, 2D_planar or 3D
        self.soil_profile = soil_profile
        self.barrier = barrier
        self.motion = motion
        self.width = width  # m
        self.height = height  # m
