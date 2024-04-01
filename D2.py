from abaqus import *
from abaqusConstants import *
from caeModules import *
import regionToolset
import material
import sketch
import part
import section
import mesh
from odbAccess import *
from numpy import float64, cumsum, array, arange, pi
import numpy as np
from data import read_json


class Create_Model:
    def __init__(self):
        self.input_obj = read_json("input.json")

        self.inf_size_x = 10  # m
        self.inf_size_y = 10  # m

        self.soil_profile = self.input_obj.soil_profile
        self.motion = self.input_obj.motion
        self.barriers = self.input_obj.barrier
        self.thicknesses = [layer.thickness for layer in self.soil_profile.layers]

        self.width = self.input_obj.width + self.inf_size_x
        self.height = sum(self.thicknesses) + self.inf_size_y
        self.source_size = float(self.motion.source_size)
        self.mesh_size = self.input_obj.mesh_size
        self.model_name = str(self.input_obj.model_name)

        self.layer_heights = np.append(
            [self.height], self.height - cumsum(self.thicknesses)
        )
        self.sorted_heights = array(sorted(self.layer_heights)[1:])

        self.f = lambda L, x: [L[i : i + 1] for i in x]

        session.viewports["Viewport: 1"].setValues(displayedObject=None)
        mdb.models.changeKey(fromName="Model-1", toName=self.model_name)
        self.model = mdb.models[self.model_name]

        self.finite_mesh_type = "CAX4R"
        self.infinite_mesh_type = "CINAX4"
        self.temp_inf_type = "CAX4I"

    def create_part(self):
        "Creates the soil profile part with additional space at right and bottom for infinite elements"

        sketch = self.model.ConstrainedSketch(
            name="__profile__", sheetSize=self.width * 2
        )
        sketch.sketchOptions.setValues(viewStyle=AXISYM)
        sketch.ConstructionLine(point1=(0.0, -100.0), point2=(0.0, 100.0))
        sketch.FixedConstraint(entity=sketch.geometry[2])
        sketch.rectangle(point1=(0, 0), point2=(self.width, self.height))
        sketch.CoincidentConstraint(
            addUndoState=False,
            entity1=sketch.vertices[0],
            entity2=sketch.geometry[2],
        )
        self.soil_part = self.model.Part(
            name="Soil Part", dimensionality=AXISYMMETRIC, type=DEFORMABLE_BODY
        )
        self.soil_part.BaseShell(sketch=sketch)
        self.soil_part.Set(faces=self.f(self.soil_part.faces, [0]), name="All Face")
        del self.model.sketches["__profile__"]

        self.model.ConstrainedSketch(
            gridSpacing=3.53,
            name="__profile__",
            sheetSize=self.width * 2,
            transform=self.soil_part.MakeSketchTransform(
                sketchPlane=self.soil_part.faces[0],
                sketchPlaneSide=SIDE1,
                sketchOrientation=RIGHT,
                origin=(0, 0, 0.0),
            ),
        )

        self.soil_part.projectReferencesOntoSketch(
            filter=COPLANAR_EDGES, sketch=self.model.sketches["__profile__"]
        )
        self.model.sketches["__profile__"].rectangle(
            point1=(0, self.height),
            point2=(self.width - self.inf_size_x, self.inf_size_y),
        )
        # self.model.sketches['__profile__'].rectangle(point1=(self.width, 0),point2=(self.width - self.inf_size_x, self.inf_size_y))
        self.model.sketches["__profile__"].Line(
            point1=(self.width - self.inf_size_x, self.inf_size_y),
            point2=(self.width, 0),
        )
        self.model.sketches["__profile__"].Line(
            point1=(self.width - self.inf_size_x - self.mesh_size, self.inf_size_y),
            point2=(self.width - self.inf_size_x - self.mesh_size, 0),
        )
        self.model.sketches["__profile__"].Line(
            point1=(self.width - self.inf_size_x, self.inf_size_y + self.mesh_size),
            point2=(self.width, self.inf_size_y + self.mesh_size),
        )
        self.soil_part.PartitionFaceBySketch(
            faces=self.soil_part.faces[0], sketch=self.model.sketches["__profile__"]
        )
        del self.model.sketches["__profile__"]

    def sketch_face(self, y, left_x):
        face = self.soil_part.faces.findAt((0.01 + left_x, y - 0.01, 0))
        self.model.ConstrainedSketch(
            gridSpacing=3.53,
            name="__profile__",
            sheetSize=self.width,
            transform=self.soil_part.MakeSketchTransform(
                sketchPlane=face,
                sketchPlaneSide=SIDE1,
                sketchOrientation=RIGHT,
                origin=(0, 0, 0.0),
            ),
        )

        self.soil_part.projectReferencesOntoSketch(
            filter=COPLANAR_EDGES, sketch=self.model.sketches["__profile__"]
        )
        self.model.sketches["__profile__"].Line(
            point1=(0 + left_x, y), point2=(self.width - self.inf_size_x + left_x, y)
        )
        self.soil_part.PartitionFaceBySketch(
            faces=face, sketch=self.model.sketches["__profile__"]
        )
        del self.model.sketches["__profile__"]

    def partition_part(self, thickness):
        offset = self.height - thickness
        if offset > self.inf_size_y:
            self.sketch_face(offset, 0)
            self.sketch_face(offset, self.width - self.inf_size_x)

    def rayleigh_damping(self, modes, Vs, H, damping_ratio):
        wn = lambda mode, Vs, H: (2 * mode - 1) * Vs / (4 * H)
        try:
            wi = wn(modes[0], Vs, H)
            wj = wn(modes[1], Vs, H)
            alpha = damping_ratio * 2 * wi * wj / (wi + wj)
            beta = damping_ratio * 2 / (wi + wj)
        except:
            alpha, beta = 0, 0

        # alpha = float64(temp_alpha)
        # beta = float64(temp_beta)
        return alpha, beta

    def create_material(self, properties):
        elastic = (properties["E"], properties["poison_ratio"])
        density = properties["D"]

        damping = properties.get("Damping", 0)
        H = properties.get("H", 0.005)
        Vs = properties.get("Vs", 0)
        alpha, beta = self.rayleigh_damping([1, 4], Vs, H, damping)

        material = self.model.Material(properties["name"])
        material.Density(table=((density,),))
        material.Elastic(table=(elastic,))
        material.Damping(alpha=alpha, beta=beta)

    def select_face(self, x, y):
        return self.soil_part.faces[self.soil_part.faces.findAt((x, y, 0)).index]

    def create_section(self, coordinate_list, section_name, material_name):
        regions = ()
        for coordinate in coordinate_list:
            regions += (self.select_face(coordinate[0], coordinate[1]),)

        self.model.HomogeneousSolidSection(name=section_name, material=material_name)
        self.soil_part.SectionAssignment(
            region=regions,
            sectionName=section_name,
        )

    def create_set(self, coordinate_list, name):
        indexes = []
        for coordinate in coordinate_list:
            indexes.append(self.select_face(coordinate[0], coordinate[1]).index)

        self.soil_part.Set(faces=self.f(self.soil_part.faces, indexes), name=name)

    def create_soil_layers(self):
        "Partition the soil profile into layers and assign material properties to each layer"
        layers = self.soil_profile.layers
        for i, layer in enumerate(layers):
            height = sum(self.thicknesses[: i + 1])

            layer_name = str(layer.layer_name)

            self.partition_part(height)
            self.create_material(
                {
                    "name": layer_name,
                    "E": layer.elastic_modulus,
                    "D": layer.density,
                    "Damping": layer.damping_ratio,
                    "Vs": layer.shear_wave_velocity,
                    "H": layer.thickness,
                    "poison_ratio": layer.poisson_ratio,
                },
            )
            cords = [
                (
                    1,
                    self.layer_heights[i] - 0.1,
                ),  # coordinates of the finite element part of the layer
                (
                    self.width - self.inf_size_x + 1,
                    self.layer_heights[i] - 0.1,
                ),  # coordinates of the infinite element part of the layer
            ]
            self.create_section(cords, layer_name, layer_name)
            self.create_set(cords[:1], layer_name + "_face")

        # assign last material to bottom side of infinite elements
        inf_bottom_coords = [
            (1, 1),
            (self.width - self.inf_size_x, 1),
            (self.width - 0.01, self.inf_size_y),
        ]
        self.create_section(
            inf_bottom_coords, "Infinite_Section", str(layers[-1].layer_name)
        )

    def create_edge_set(self):
        "Create sets of edges for boundary conditions and mesh control"
        left_finite_edges = []
        finite_horizontal_edges = []
        vertical_edges = []
        single_seed_edges = []
        single_seed_coordinates = [
            (0, 0.01, 0),
            (self.width - self.inf_size_x - self.mesh_size, 0.01, 0),
            (self.width - 0.01, self.height, 0),
            (self.width - 0.01, self.inf_size_y + self.mesh_size, 0),
            (self.width - 0.01, 0, 0),
            (self.width, 0.01, 0),
            ((self.width - self.inf_size_x / 2), self.inf_size_y / 2, 0),
        ]
        for i in single_seed_coordinates:
            single_seed_edges.append(self.soil_part.edges.findAt(i).index)

        y_coordinates = list(self.layer_heights)
        for i in range(len(y_coordinates)):
            y = y_coordinates[i]
            finite_edge_id = self.soil_part.edges.findAt((0.01, y, 0)).index
            finite_horizontal_edges.append(finite_edge_id)

        for y in arange(self.height - 0.01, self.inf_size_y + self.mesh_size, -0.1):
            N1 = (0, y, 0)
            N2 = (self.width - self.inf_size_x, y, 0)
            N3 = (self.width, y, 0)
            try:
                id1 = self.soil_part.edges.findAt(N1).index
                id2 = self.soil_part.edges.findAt(N2).index
                id3 = self.soil_part.edges.findAt(N3).index

                left_finite_edges.append(id1)
                vertical_edges.append(id1)
                vertical_edges.append(id2)
                vertical_edges.append(id3)
            except:
                pass

        self.soil_part.Set(
            edges=self.f(self.soil_part.edges, left_finite_edges),
            name="Left_Vertical_Edges",
        )
        self.soil_part.Set(
            edges=self.f(self.soil_part.edges, vertical_edges),
            name="All_Vertical_Edges",
        )
        self.soil_part.Set(
            edges=self.f(self.soil_part.edges, finite_horizontal_edges),
            name="Finite_Horizontal_Edges",
        )
        self.soil_part.Set(
            edges=self.f(self.soil_part.edges, single_seed_edges),
            name="Single_Seed_Edges",
        )

    def draw_source(self):
        "Draws the source of the vibration and creates a set of edges for the source"
        N1 = (0, self.height, 0)
        N2 = (self.source_size, self.height, 0)

        self.soil_part.DatumPointByCoordinate((0, self.height, 0))
        self.soil_part.DatumPointByCoordinate((self.source_size, self.height, 0))

        top_part = self.soil_part.faces.findAt((0.01, self.height, 0))
        self.soil_part.PartitionFaceByShortestPath(faces=top_part, point1=N1, point2=N2)
        id1 = self.soil_part.edges.findAt(
            (0.5 * self.source_size, self.height, 0)
        ).index
        self.soil_part.Set(edges=self.f(self.soil_part.edges, [id1]), name="Source")

    def create_instance(self):
        self.model.rootAssembly.DatumCsysByDefault(CARTESIAN)
        self.model.rootAssembly.Instance(
            dependent=ON, name="Soil Part-1", part=self.soil_part
        )
        self.model.rootAssembly.regenerate()

    def create_sketch(self, x, y):
        face = self.soil_part.faces.findAt((x, y, 0))
        self.model.ConstrainedSketch(
            gridSpacing=3.53,
            name="__profile__",
            sheetSize=self.width,
            transform=self.soil_part.MakeSketchTransform(
                sketchPlane=face,
                sketchPlaneSide=SIDE1,
                sketchOrientation=RIGHT,
                origin=(0, 0, 0.0),
            ),
        )

        self.soil_part.projectReferencesOntoSketch(
            filter=COPLANAR_EDGES, sketch=self.model.sketches["__profile__"]
        )

        return self.model.sketches["__profile__"]

    def create_barriers(self):
        for barrier in self.barriers:
            x = barrier.distance_to_source + self.source_size
            barrier_width = barrier.width
            barrier_height = barrier.height
            barrier_name = str(barrier.barrier_name)
            for i, y in enumerate(self.layer_heights[:-1]):
                if y > self.height - barrier.height:
                    sketch = self.create_sketch(0.1, y - 0.01)
                    if self.layer_heights[i + 1] > self.height - barrier.height:
                        y2 = self.layer_heights[i + 1]
                    else:
                        y2 = self.height - barrier_height

                    sketch.rectangle(
                        point1=(x, y),
                        point2=(x + barrier_width, y2),
                    )
                    face = self.select_face(x + 0.01, y2 + 0.1)

                    if barrier.density == 0:
                        self.soil_part.Cut(sketch=sketch)
                    else:
                        self.create_material(
                            {
                                "name": barrier_name,
                                "E": barrier.elastic_modulus,
                                "D": barrier.density,
                                "Damping": barrier.damping_ratio,
                                "Vs": barrier.shear_wave_velocity,
                                "H": barrier.width,
                                "poison_ratio": barrier.poisson_ratio,
                            },
                        )
                        self.soil_part.PartitionFaceBySketch(
                            faces=face, sketch=self.model.sketches["__profile__"]
                        )

                        section_name = barrier_name + "_section"
                        self.model.HomogeneousSolidSection(
                            name=section_name, material=barrier_name
                        )
                        face = self.select_face(x + 0.01, y2 + 0.1)
                        self.soil_part.SectionAssignment(
                            region=(face,),
                            sectionName=section_name,
                        )
                    del sketch

    def create_face_sets(self):
        "Create sets of faces for mesh control"
        finite_faces = self.soil_part.faces.getByBoundingBox(
            0, self.inf_size_y, 0, self.width - self.inf_size_x, self.height, 0
        )
        self.soil_part.Set(faces=finite_faces, name="Finite_Faces")

        infinite_faces = []
        for face in self.soil_part.faces:
            if face not in finite_faces:
                infinite_faces.append(face.index)

        self.soil_part.Set(
            faces=self.f(self.soil_part.faces, infinite_faces), name="Infinite_Faces"
        )

    def set_mesh_control(self):
        face1 = self.select_face(self.width - 1, 0.01)
        face2 = self.select_face(self.width - 1, self.inf_size_y / 2)
        diagonal_edge = self.soil_part.edges.findAt(
            (self.width - self.inf_size_x / 2, self.inf_size_y / 2, 0)
        )
        self.soil_part.setSweepPath(edge=diagonal_edge, region=face1, sense=FORWARD)
        self.soil_part.setSweepPath(edge=diagonal_edge, region=face2, sense=FORWARD)
        self.soil_part.setMeshControls(
            regions=self.soil_part.sets["Infinite_Faces"].faces, technique=SWEEP
        )

        for y in self.layer_heights[:-1]:
            face = self.select_face(self.width - 0.01, y - 0.01)
            edge = self.soil_part.edges.findAt((self.width - 0.01, y, 0))
            self.soil_part.setSweepPath(edge=edge, region=face, sense=FORWARD)

        try:
            if self.height - self.thrench_depth not in self.layer_heights:
                y = self.height - self.thrench_depth
                face = self.select_face(self.width - 0.01, y - 0.01)
                edge = self.soil_part.edges.findAt((self.width - 0.01, y, 0))
                self.soil_part.setSweepPath(edge=edge, region=face, sense=FORWARD)
        except:
            pass

    def create_mesh(self):
        self.set_mesh_control()
        self.soil_part.setElementType(
            regions=(self.soil_part.sets["Infinite_Faces"]),
            elemTypes=(
                mesh.ElemType(elemCode=CAX4I, elemLibrary=STANDARD),
                mesh.ElemType(elemCode=CAX4I, elemLibrary=STANDARD),
            ),
        )

        self.soil_part.seedEdgeBySize(
            edges=self.soil_part.sets["All_Vertical_Edges"].edges,
            size=self.mesh_size,
            constraint=FIXED,
        )
        self.soil_part.seedEdgeBySize(
            edges=self.soil_part.sets["Finite_Horizontal_Edges"].edges,
            size=self.mesh_size,
            constraint=FIXED,
        )
        self.soil_part.seedEdgeBySize(
            edges=(self.soil_part.edges.findAt((0.01, 0, 0)),), size=self.mesh_size
        )
        self.soil_part.seedEdgeByNumber(
            edges=self.soil_part.sets["Single_Seed_Edges"].edges,
            number=1,
            constraint=FIXED,
        )

        self.soil_part.generateMesh()

    def create_nodes(self):
        self.node_list = []
        for i in self.motion.accelerometer_pattern:
            self.node_list.append(
                self.soil_part.nodes.getClosest(
                    (float(i) + self.source_size, self.height, 0)
                )
            )
            self.node_list.append(
                self.soil_part.nodes.getClosest(
                    (float(i) + self.source_size, self.height, 0)
                )
            )

        self.soil_part.Set(
            nodes=mesh.MeshNodeArray(self.node_list), name="Accelerometers"
        )

    def create_vibration(self, time_step, duration, frequency, amplitude):
        time = arange(0, duration + time_step, time_step)
        accelerations = amplitude * np.sin(2 * pi * frequency * time)
        self.vibration_data = [[time[i], accelerations[i]] for i in range(len(time))]

    def create_step(self):
        time_step = self.motion.time_step
        duration = self.motion.duration
        frequency = self.motion.frequency
        amplitude = self.motion.amplitude

        self.create_vibration(time_step, duration, frequency, amplitude)
        self.model.ImplicitDynamicsStep(
            initialInc=time_step,
            timePeriod=duration,
            maxNumInc=int(2 * (duration / time_step)),
            name="Vibration Step",
            previous="Initial",
            maxInc=time_step,
        )
        self.model.TabularAmplitude(
            name="Vibration",
            timeSpan=STEP,
            smooth=SOLVER_DEFAULT,
            data=self.vibration_data,
        )

    def create_boundary_conditions(self):
        sets = self.model.rootAssembly.instances["Soil Part-1"].sets
        self.model.XsymmBC(
            createStepName="Initial",
            name="BC-X-Left",
            region=sets["Left_Vertical_Edges"],
        )

        source_edge = self.soil_part.edges.findAt(
            (self.source_size / 2, self.height, 0)
        ).index
        self.soil_part.Surface(
            name="Source_Top",
            side1Edges=self.f(
                self.soil_part.edges,
                [
                    source_edge,
                ],
            ),
        )
        self.model.Pressure(
            amplitude="Vibration",
            createStepName="Vibration Step",
            magnitude=1,
            name="Vibration",
            region=self.model.rootAssembly.instances["Soil Part-1"].surfaces[
                "Source_Top"
            ],
        )
        # self.model.AccelerationBC(a2=-1, amplitude='Vibration',createStepName='Vibration Step', fieldName='',name='BC-2',region=Sets["Source"])

    def create_history_output(self):
        self.model.fieldOutputRequests["F-Output-1"].deactivate("Vibration Step")
        self.model.HistoryOutputRequest(
            createStepName="Vibration Step",
            frequency=1,
            name="H-Output-2",
            variables=("A2", "V2"),
            region=self.model.rootAssembly.allInstances["Soil Part-1"].sets[
                "Accelerometers"
            ],
        )

        del self.model.historyOutputRequests["H-Output-1"]
        del self.model.fieldOutputRequests["F-Output-1"]

    def create_job(self):
        mdb.Job(
            model=self.model_name,
            name=self.model_name,
            type=ANALYSIS,
            memory=90,
            memoryUnits=PERCENTAGE,
        )
        mdb.jobs[self.model_name].writeInput(consistencyChecking=OFF)

    def change_element_type(self):
        inp = open(self.model_name + ".inp")
        data = inp.read().replace(self.temp_inf_type, self.infinite_mesh_type)
        new_inp = open(self.model_name + ".inp", "w")
        new_inp.write(data)
        new_inp.close()
        inp.close()

    def operator(self):
        self.create_part()
        self.create_soil_layers()
        self.create_edge_set()
        self.draw_source()
        self.create_instance()
        self.create_barriers()
        self.create_face_sets()
        self.create_mesh()
        self.create_nodes()
        self.create_step()
        self.create_boundary_conditions()
        self.create_history_output()
        self.create_job()
        self.change_element_type()


model = Create_Model()
model.operator()
