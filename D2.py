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
from numpy import float64,cumsum,array,arange,pi
import numpy as np
from helper import read_json
from model import Input

class Create_Model:
    def __init__(self):
        self.input_obj = read_json("input.json")

        self.inf_size_x = 10 # m
        self.inf_size_y = 10 # m

        self.soil_profile = self.input_obj.soil_profile
        self.motion = self.input_obj.motion
        self.barrier = self.input_obj.barrier
        
        self.Width = float64(self.input_obj.width)+self.inf_size_x
        self.Height = float64(sum(self.input_obj.height))+self.inf_size_y
        self.source_size = float64(self.input_obj.source_size)
        self.accelometer_pattern = temp_accelerometer_pattern
        self.PGA = temp_PGA
        self.duration = temp_duration
        self.frequency = temp_frequency
        self.time_step = temp_time_step

        self.ditch_width = temp_ditch_width
        self.ditch_depth = temp_ditch_depth
        self.ditch2source = temp_ditch2source
        self.ditchnumber = temp_ditchnumber
        self.ditch2ditch = temp_ditch2ditch

        self.fillDitch = int(temp_fill_ditch)
        self.RC_E = temp_RC_E
        self.RC_density = temp_RC_density
        self.RC_damping = temp_RC_damping
        self.RC_VS = temp_RC_VS

        self.SP_pattern = array([temp_SP_pattern])+self.source_size
        self.SP_E = temp_SP_E
        self.SP_density = temp_SP_density
        self.SP_thickness = temp_SP_thickness
        self.SP_height = temp_SP_height
        self.SP_interaction = temp_SP_interaction


        self.Thicknesses = self.parameter["Thicknesses"]
        self.layer_heights = np.append([self.Height], self.Height - cumsum(self.parameter["Thicknesses"]))
        self.sorted_heights = array(sorted(self.layer_heights)[1:])

        self.model_size = sum(self.accelometer_pattern) + self.source_size
        self.model_name = "temp_model_name"

        self.mesh_size = temp_mesh_size
        self.f = lambda L, x: [L[i:i + 1] for i in x]
        session.viewports["Viewport: 1"].setValues(displayedObject=None)
        mdb.models.changeKey(fromName="Model-1", toName=self.model_name)
        self.soilModel = mdb.models[self.model_name]

        self.finite_mesh_type = "CAX4R"
        self.infinite_mesh_type = "CINAX4"
        self.temp_inf_type = "CAX4I"

    def create_part(self):
        soilProfileSketch = self.soilModel.ConstrainedSketch(name="__profile__", sheetSize=self.Width * 2)
        soilProfileSketch.sketchOptions.setValues(viewStyle=AXISYM)
        soilProfileSketch.ConstructionLine(point1=(0.0, -100.0), point2=(0.0, 100.0))
        soilProfileSketch.FixedConstraint(entity=soilProfileSketch.geometry[2])
        soilProfileSketch.rectangle(point1=(0, 0), point2=(self.Width, self.Height))
        soilProfileSketch.CoincidentConstraint(addUndoState=False, entity1=soilProfileSketch.vertices[0],
                                               entity2=soilProfileSketch.geometry[2])
        self.soilPart = self.soilModel.Part(name="Soil Part", dimensionality=AXISYMMETRIC, type=DEFORMABLE_BODY)
        self.soilPart.BaseShell(sketch=soilProfileSketch)
        self.soilPart.Set(faces=self.f(self.soilPart.faces,[0]),name="All Face")
        del self.soilModel.sketches['__profile__']

        self.soilModel.ConstrainedSketch(gridSpacing=3.53, name='__profile__', sheetSize=self.Width * 2,
                                         transform=self.soilPart.MakeSketchTransform(
                                             sketchPlane=self.soilPart.faces[0],
                                             sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(0, 0, 0.0)))

        self.soilPart.projectReferencesOntoSketch(filter=COPLANAR_EDGES,
                                                  sketch=self.soilModel.sketches['__profile__'])
        self.soilModel.sketches['__profile__'].rectangle(point1=(0, self.Height),
                                                         point2=(self.Width - self.inf_size_x, self.inf_size_y))
        # self.soilModel.sketches['__profile__'].rectangle(point1=(self.Width, 0),point2=(self.Width - self.inf_size_x, self.inf_size_y))
        self.soilModel.sketches['__profile__'].Line(point1=(self.Width - self.inf_size_x, self.inf_size_y),
                                                    point2=(self.Width, 0))
        self.soilModel.sketches['__profile__'].Line(
            point1=(self.Width - self.inf_size_x - self.mesh_size, self.inf_size_y),
            point2=(self.Width - self.inf_size_x - self.mesh_size, 0))
        self.soilModel.sketches['__profile__'].Line(
            point1=(self.Width - self.inf_size_x, self.inf_size_y + self.mesh_size),
            point2=(self.Width, self.inf_size_y + self.mesh_size))
        self.soilPart.PartitionFaceBySketch(faces=self.soilPart.faces[0],
                                            sketch=self.soilModel.sketches['__profile__'])
        del self.soilModel.sketches['__profile__']

    def create_sheet_pile(self):
        if len(self.SP_pattern) > 0:
            self.soilMaterial = self.soilModel.Material("Steel")
            self.soilMaterial.Density(table=((self.SP_density,),))
            self.soilMaterial.Elastic(table=((self.SP_E,0.33),))
            #alpha,beta = self.rayleigh_damping([1,4],3250,self.SP_height,0.02)
            #self.soilMaterial.Damping(alpha=alpha, beta=beta)
            self.soilModel.HomogeneousSolidSection(name="Sheet Pile Section", material="Steel")

            try:
                for x in self.SP_pattern:
                    face = self.soilPart.faces.findAt((0.01, self.Height - self.SP_height, 0))
                    self.soilModel.ConstrainedSketch(gridSpacing=3.53, name='__profile__', sheetSize=self.Width,
                                                        transform=self.soilPart.MakeSketchTransform(sketchPlane=face,
                                                                                                    sketchPlaneSide=SIDE1,
                                                                                                    sketchOrientation=RIGHT,
                                                                                                    origin=(0, 0, 0.0)))
                    self.soilPart.projectReferencesOntoSketch(filter=COPLANAR_EDGES,
                                                                sketch=self.soilModel.sketches['__profile__'])
                    self.soilModel.sketches['__profile__'].Line(point1=(0, self.Height - self.SP_height),
                                                                point2=(self.Width, self.Height - self.SP_height))
                    self.soilPart.PartitionFaceBySketch(faces=face, sketch=self.soilModel.sketches['__profile__'])
                    del self.soilModel.sketches['__profile__']
            except:
                pass

            for y in list(self.layer_heights[:-1])+[self.Height - self.SP_height]:
                for i in range(len(self.SP_pattern)):
                    #Draw Sheet Pile
                    face = self.soilPart.faces.findAt((self.Width - self.inf_size_x - 0.01,y-0.01,0))
                    self.soilModel.ConstrainedSketch(gridSpacing=3.53, name='__profile__', sheetSize=self.Width,
                                                    transform=self.soilPart.MakeSketchTransform(sketchPlane=face,
                                                                                                sketchPlaneSide=SIDE1,
                                                                                                sketchOrientation=RIGHT,
                                                                                                origin=(0, 0, 0.0)))

                    self.soilPart.projectReferencesOntoSketch(filter=COPLANAR_EDGES,
                                                            sketch=self.soilModel.sketches['__profile__'])
                    self.soilModel.sketches['__profile__'].rectangle(point1=(self.SP_pattern[i], y),
                                                                    point2=(self.SP_pattern[i]+self.SP_thickness, 0))
                    try:
                        self.soilPart.PartitionFaceBySketch(faces=face,sketch=self.soilModel.sketches['__profile__'])
                    except:
                        pass
                    del self.soilModel.sketches['__profile__']
    
            #Section Assignment
            for y in self.layer_heights:
                for i in range(len(self.SP_pattern)):               
                    if y > self.Height - self.SP_height:
                        face = self.soilPart.faces.findAt((self.SP_pattern[i]+self.SP_thickness/2,y-0.01,0))
                        self.soilPart.SectionAssignment(region=(face,),sectionName="Sheet Pile Section")
            face = self.soilPart.faces.findAt((self.SP_pattern[i]+self.SP_thickness/2,self.Height - self.SP_height + 0.01,0))
            self.soilPart.SectionAssignment(region=(face,),sectionName="Sheet Pile Section")

    def create_rubber_barrier(self):
        if self.fillDitch and self.ditchnumber:
            alpha,beta = self.rayleigh_damping([1,4],self.RC_VS,self.ditch_depth,self.RC_damping)
            self.soilMaterial = self.soilModel.Material("Rubber")
            self.soilMaterial.Density(table=((self.RC_density,),))
            self.soilMaterial.Elastic(table=((self.RC_E,0.25),))
            self.soilMaterial.Damping(alpha=alpha, beta=beta)
            self.soilModel.HomogeneousSolidSection(name="Rubber Chip Section", material="Rubber")

            try:
                for i in range(self.ditchnumber):
                    face = self.soilPart.faces.findAt((0.01, self.Height - self.ditch_depth, 0))
                    self.soilModel.ConstrainedSketch(gridSpacing=3.53, name='__profile__', sheetSize=self.Width,
                                                        transform=self.soilPart.MakeSketchTransform(sketchPlane=face,
                                                                                                    sketchPlaneSide=SIDE1,
                                                                                                    sketchOrientation=RIGHT,
                                                                                                    origin=(0, 0, 0.0)))
                    self.soilPart.projectReferencesOntoSketch(filter=COPLANAR_EDGES,
                                                                sketch=self.soilModel.sketches['__profile__'])
                    self.soilModel.sketches['__profile__'].Line(point1=(0, self.Height - self.ditch_depth),
                                                                point2=(self.Width, self.Height - self.ditch_depth))
                    self.soilPart.PartitionFaceBySketch(faces=face, sketch=self.soilModel.sketches['__profile__'])
                    del self.soilModel.sketches['__profile__']
            except:
                pass

            for i in range(self.ditchnumber):
                x = self.source_size + self.ditch2source + i*(self.ditch_width + self.ditch2ditch)
                face = self.soilPart.faces.findAt((x +  0.01,self.Height,0))
                self.soilModel.ConstrainedSketch(gridSpacing=3.53, name='__profile__', sheetSize=self.Width,
                                                transform=self.soilPart.MakeSketchTransform(sketchPlane=face,
                                                                                            sketchPlaneSide=SIDE1,
                                                                                            sketchOrientation=RIGHT,
                                                                                            origin=(0, 0, 0.0)))

                self.soilPart.projectReferencesOntoSketch(filter=COPLANAR_EDGES,
                                                        sketch=self.soilModel.sketches['__profile__'])
                self.soilModel.sketches['__profile__'].rectangle(point1=(x,self.Height - self.ditch_depth),
                                                                point2=(x+self.ditch_width, self.Height))
                self.soilPart.PartitionFaceBySketch(faces=face,sketch=self.soilModel.sketches['__profile__'])
                del self.soilModel.sketches['__profile__']
    
            #Section Assignment
            for y in self.layer_heights:
                for i in range(self.ditchnumber):
                    x = self.source_size + self.ditch2source + i*(self.ditch_width + self.ditch2ditch)             
                    if y >= self.Height - self.ditch_depth:
                        face = self.soilPart.faces.findAt((x+self.ditch_width/2,y-0.01,0))
                        self.soilPart.SectionAssignment(region=(face,),sectionName="Rubber Chip Section")
            face = self.soilPart.faces.findAt((x+self.ditch_width/2,self.Height - self.ditch_depth + 0.01,0))
            self.soilPart.SectionAssignment(region=(face,),sectionName="Rubber Chip Section")
               
    def create_ditch(self):
        if self.ditchnumber > 0 and not self.fillDitch:
            top_face = self.soilPart.faces.findAt((0.01, self.Height - 0.01, 0))
            for i in range(self.ditchnumber):
                x = self.source_size + self.ditch2source + i*(self.ditch_width + self.ditch2ditch)
                self.soilModel.ConstrainedSketch(gridSpacing=3.53, name='__profile__', sheetSize=self.Width,
                                                 transform=self.soilPart.MakeSketchTransform(sketchPlane=top_face,
                                                                                             sketchPlaneSide=SIDE1,
                                                                                             sketchOrientation=RIGHT,
                                                                                             origin=(0, 0, 0.0)))

                self.soilPart.projectReferencesOntoSketch(filter=COPLANAR_EDGES,sketch=self.soilModel.sketches['__profile__'])
                self.soilModel.sketches['__profile__'].rectangle(point1=(x, self.Height),
                                                            point2=(x + self.ditch_width, self.Height - self.ditch_depth))
                
                self.soilPart.Cut(sketch=self.soilModel.sketches['__profile__'])
                del self.soilModel.sketches['__profile__']

            try:
                for x in [0.01]+list(self.SP_pattern+self.SP_thickness+0.01)+[self.Width-0.01]:
                    face = self.soilPart.faces.findAt((x, self.Height - self.ditch_depth, 0))
                    self.soilModel.ConstrainedSketch(gridSpacing=3.53, name='__profile__', sheetSize=self.Width,
                                                     transform=self.soilPart.MakeSketchTransform(sketchPlane=face,
                                                                                                 sketchPlaneSide=SIDE1,
                                                                                                 sketchOrientation=RIGHT,
                                                                                                 origin=(0, 0, 0.0)))
                    self.soilPart.projectReferencesOntoSketch(filter=COPLANAR_EDGES,sketch=self.soilModel.sketches['__profile__'])
                    self.soilModel.sketches['__profile__'].Line(point1=(0, self.Height - self.ditch_depth),
                                                                     point2=(self.Width,self.Height - self.ditch_depth))
                    self.soilPart.PartitionFaceBySketch(faces=face, sketch=self.soilModel.sketches['__profile__'])
                    del self.soilModel.sketches['__profile__']
            except:
                pass

    def sketch_face(self, y, left_x):
        face = self.soilPart.faces.findAt((0.01 + left_x, y - 0.01, 0))
        self.soilModel.ConstrainedSketch(gridSpacing=3.53, name='__profile__', sheetSize=self.Width,
                                         transform=self.soilPart.MakeSketchTransform(sketchPlane=face,
                                                                                     sketchPlaneSide=SIDE1,
                                                                                     sketchOrientation=RIGHT,
                                                                                     origin=(0, 0, 0.0)))

        self.soilPart.projectReferencesOntoSketch(filter=COPLANAR_EDGES,
                                                  sketch=self.soilModel.sketches['__profile__'])
        self.soilModel.sketches['__profile__'].Line(point1=(0 + left_x, y),
                                                    point2=(self.Width - self.inf_size_x + left_x, y))
        self.soilPart.PartitionFaceBySketch(faces=face, sketch=self.soilModel.sketches['__profile__'])
        del self.soilModel.sketches['__profile__']

    def partition_part(self, thickness):
        offset = self.Height - thickness
        if offset > self.inf_size_y:
            self.sketch_face(offset, 0)
            self.sketch_face(offset, self.Width - self.inf_size_x)

    def create_material(self, name, properties):
        elastic = properties["E"]
        density = properties["D"]

        damping = properties.get("Damping", 0)
        H = properties.get("H", 0.005)
        Vs = properties.get("Vs", 0)
        alpha, beta = self.rayleigh_damping([1, 4], Vs, H, damping)
        #alpha, beta = float64(temp_alpha),float64(temp_beta)

        self.soilMaterial = self.soilModel.Material(name)
        self.soilMaterial.Density(table=((density,),))
        self.soilMaterial.Elastic(table=(elastic,))
        self.soilMaterial.Damping(alpha=alpha, beta=beta)

    def create_section(self):
        bottom_face = self.soilPart.faces[self.soilPart.faces.findAt((0.01, 0.01, 0)).index]
        bottom_right_face = self.soilPart.faces[self.soilPart.faces.findAt((self.Width - 0.01, 0, 0)).index]
        right_bottom_face = self.soilPart.faces[self.soilPart.faces.findAt((self.Width - 0.01, self.inf_size_y - 0.01, 0)).index]
        last_inf_face_left = self.soilPart.faces[self.soilPart.faces.findAt((0.01, self.inf_size_y+0.01, 0)).index]
        last_inf_face_right = self.soilPart.faces[self.soilPart.faces.findAt((self.Width - 0.01, self.inf_size_y+self.mesh_size+0.01, 0)).index]

        self.soilModel.HomogeneousSolidSection(name="Infinite_Section", material=self.parameter["Name"][-1])
        self.soilPart.SectionAssignment(region=(bottom_face, bottom_right_face, right_bottom_face,last_inf_face_left,last_inf_face_right),sectionName="Infinite_Section")

        for i in range(len(self.layer_heights) - 1):
            y = float64(self.layer_heights[i]) - 0.01
            name = self.parameter["Name"][i]

            index1 = self.soilPart.faces.findAt((0.01, y, 0)).index
            index2 = self.soilPart.faces.findAt((self.Width - 0.01, y, 0)).index
            section_name = name + "_Section"

            self.soilModel.HomogeneousSolidSection(name=section_name, material=name)
            self.soilPart.SectionAssignment(region=(self.soilPart.faces[index1], self.soilPart.faces[index2]),
                                            sectionName=section_name)
            index_list = [index1]
            for x in self.SP_pattern:
                index = self.soilPart.faces.findAt((x+self.SP_thickness+0.01,y,0)).index
                self.soilPart.SectionAssignment(region=(self.soilPart.faces[index], ),sectionName=section_name)
                index_list.append(index)
            self.soilPart.Set(faces=self.f(self.soilPart.faces, index_list), name=name + "_Face")

    def create_face_sets(self):
        finite_faces = self.soilPart.faces.getByBoundingBox(0, self.inf_size_y, 0, self.Width - self.inf_size_x,
                                                            self.Height, 0)
        self.soilPart.Set(faces=finite_faces, name="Finite_Faces")

        inf_face1 = self.soilPart.faces.findAt((0.01, 0.01, 0)).index
        inf_face2 = self.soilPart.faces.findAt((self.Width - self.inf_size_x + 0.01, 0.01, 0)).index
        inf_face3 = self.soilPart.faces.findAt((self.Width - 0.01, self.inf_size_y / 2, 0)).index
        infinite_faces = [inf_face1, inf_face2, inf_face3]
        for i in range(len(self.layer_heights)):
            y = float64(self.layer_heights[i] - 0.01)
            infinite_faces.append(self.soilPart.faces.findAt((self.Width - 0.01, y, 0)).index)
        self.soilPart.Set(faces=self.f(self.soilPart.faces, infinite_faces), name="Infinite_Faces")

    def create_edge_set(self):
        left_finite_edges = []
        finite_horizontal_edges = []
        vertical_edges = []
        single_seed_edges = []
        single_seed_coordinates = [(0, 0.01, 0), (self.Width - self.inf_size_x - self.mesh_size, 0.01, 0),
                                   (self.Width - 0.01, self.Height, 0),
                                   (self.Width - 0.01, self.inf_size_y + self.mesh_size, 0),
                                   (self.Width - 0.01, 0, 0), (self.Width, 0.01, 0),
                                   ((self.Width - self.inf_size_x / 2), self.inf_size_y / 2, 0)]
        for i in single_seed_coordinates:
            single_seed_edges.append(self.soilPart.edges.findAt(i).index)

        y_coordinates = list(self.layer_heights)
        for i in range(len(y_coordinates)):
            y = y_coordinates[i]
            finite_edge_id = self.soilPart.edges.findAt((0.01, y, 0)).index
            finite_horizontal_edges.append(finite_edge_id)
            for x in self.SP_pattern:
                finite_edge_id = self.soilPart.edges.findAt((x + self.SP_thickness + 0.001, y, 0)).index
                finite_horizontal_edges.append(finite_edge_id)

        for y in arange(self.Height-0.01,self.inf_size_y + self.mesh_size,-0.1):
            #y = float64(self.layer_heights[i] - 0.01)

            # Vertical Edges
            N1 = (0, y, 0)
            N2 = (self.Width - self.inf_size_x, y, 0)
            N3 = (self.Width, y, 0)
            try:
                id1 = self.soilPart.edges.findAt(N1).index
                id2 = self.soilPart.edges.findAt(N2).index
                id3 = self.soilPart.edges.findAt(N3).index

                left_finite_edges.append(id1)
                vertical_edges.append(id1)
                vertical_edges.append(id2)
                vertical_edges.append(id3)
            except:
                a=1

        self.soilPart.Set(edges=self.f(self.soilPart.edges, left_finite_edges), name="Left_Vertical_Edges")
        self.soilPart.Set(edges=self.f(self.soilPart.edges, vertical_edges), name="All_Vertical_Edges")
        self.soilPart.Set(edges=self.f(self.soilPart.edges, finite_horizontal_edges),
                          name="Finite_Horizontal_Edges")
        self.soilPart.Set(edges=self.f(self.soilPart.edges, single_seed_edges), name="Single_Seed_Edges")

    def draw_source(self):
        N1 = (0, self.Height, 0)
        N2 = (self.source_size, self.Height, 0)

        self.soilPart.DatumPointByCoordinate((0, self.Height, 0))
        self.soilPart.DatumPointByCoordinate((self.source_size, self.Height, 0))

        top_part = self.soilPart.faces.findAt((0.01, self.Height, 0))
        self.soilPart.PartitionFaceByShortestPath(faces=top_part, point1=N1, point2=N2)
        id1 = self.soilPart.edges.findAt((0.5 * self.source_size, self.Height, 0)).index
        self.soilPart.Set(edges=self.f(self.soilPart.edges, [id1]), name="Source")

    def create_instance(self):
        self.soilModel.rootAssembly.DatumCsysByDefault(CARTESIAN)
        self.soilModel.rootAssembly.Instance(dependent=ON, name="Soil Part-1", part=self.soilPart)
        self.soilModel.rootAssembly.regenerate()

    def rayleigh_damping(self, modes, Vs, H, damping_ratio):
        wn = lambda mode, Vs, H: (2 * mode - 1) * Vs / (4 * H)
        try:
            wi = wn(modes[0], Vs, H)
            wj = wn(modes[1], Vs, H)
            alpha = damping_ratio * 2 * wi * wj / (wi + wj)
            beta = damping_ratio * 2 / (wi + wj)
        except:
            alpha, beta = 0, 0

        #alpha = float64(temp_alpha)
        #beta = float64(temp_beta)
        return alpha, beta

    def create_vibration(self):
        time = arange(0, self.duration + self.time_step, self.time_step)
        accelerations = self.PGA * np.sin(2 * pi * self.frequency * time)
        self.data = [[time[i], accelerations[i]] for i in range(len(time))]

    def create_step(self):
        self.create_vibration()
        self.soilModel.ImplicitDynamicsStep(initialInc=self.time_step, timePeriod=self.duration,
                                            maxNumInc=int(2 * (self.duration / self.time_step)),
                                            name='Vibration Step',
                                            previous='Initial', maxInc=self.time_step)
        self.soilModel.TabularAmplitude(name="Vibration", timeSpan=STEP, smooth=SOLVER_DEFAULT, data=self.data)

    def create_boundary_conditions(self):
        Sets = self.soilModel.rootAssembly.instances["Soil Part-1"].sets
        self.soilModel.XsymmBC(createStepName='Initial', name='BC-X-Left', region=Sets['Left_Vertical_Edges'])
        #self.soilModel.DisplacementBC(amplitude="Vibration", createStepName='Vibration Step', name='Vibration',region=Sets['Source'], u2=-0.001)

        source_edge = self.soilPart.edges.findAt((self.source_size / 2, self.Height, 0)).index
        self.soilPart.Surface(name="Source_Top", side1Edges=self.f(self.soilPart.edges, [source_edge, ]))
        self.soilModel.Pressure(amplitude='Vibration', createStepName='Vibration Step', magnitude=1,name='Vibration',
                                region=self.soilModel.rootAssembly.instances['Soil Part-1'].surfaces['Source_Top'])
        #self.soilModel.AccelerationBC(a2=-1, amplitude='Vibration',createStepName='Vibration Step', fieldName='',name='BC-2',region=Sets["Source"])

    def set_mesh_control(self):
        face1 = self.soilPart.faces.findAt((self.Width - 1, 0.01,0))
        face2 = self.soilPart.faces.findAt((self.Width - 1, self.inf_size_y/2,0))
        diagonal_edge = self.soilPart.edges.findAt((self.Width - self.inf_size_x/2,self.inf_size_y/2,0))
        self.soilPart.setSweepPath(edge=diagonal_edge, region=face1, sense=FORWARD)
        self.soilPart.setSweepPath(edge=diagonal_edge, region=face2, sense=FORWARD)
        self.soilPart.setMeshControls(regions=self.soilPart.sets["Infinite_Faces"].faces, technique=SWEEP)

        for y in self.layer_heights[:-1]:
            face = self.soilPart.faces.findAt((self.Width - 0.01,y-0.01,0))
            edge = self.soilPart.edges.findAt((self.Width-0.01,y,0))
            self.soilPart.setSweepPath(edge=edge, region=face, sense=FORWARD)
        
        try:
            if self.Height - self.ditch_depth not in self.layer_heights:
                y = self.Height - self.ditch_depth
                face = self.soilPart.faces.findAt((self.Width - 0.01, y-0.01, 0))
                edge = self.soilPart.edges.findAt((self.Width-0.01, y, 0))
                self.soilPart.setSweepPath(edge=edge, region=face, sense=FORWARD)
        except:
            pass

    def create_mesh(self):
        self.set_mesh_control()
        self.soilPart.setElementType(regions=(self.soilPart.sets["Infinite_Faces"]),elemTypes=(
        mesh.ElemType(elemCode=CAX4I, elemLibrary=STANDARD), mesh.ElemType(elemCode=CAX4I, elemLibrary=STANDARD)))

        self.soilPart.seedEdgeBySize(edges=self.soilPart.sets["All_Vertical_Edges"].edges, size=self.mesh_size,
                                     constraint=FIXED)
        self.soilPart.seedEdgeBySize(edges=self.soilPart.sets["Finite_Horizontal_Edges"].edges, size=self.mesh_size,
                                     constraint=FIXED)
        self.soilPart.seedEdgeBySize(edges=(self.soilPart.edges.findAt((0.01, 0, 0)),), size=self.mesh_size)
        self.soilPart.seedEdgeByNumber(edges=self.soilPart.sets["Single_Seed_Edges"].edges, number=1,
                                       constraint=FIXED)

        self.soilPart.generateMesh()

    def create_nodes(self):
        self.node_list = []
        x2 = self.source_size + array(self.accelometer_pattern)
        for i in x2:
            self.node_list.append(self.soilPart.nodes.getClosest((i, self.Height, 0)))
            self.node_list.append(self.soilPart.nodes.getClosest((i, self.Height, 0)))

        self.soilPart.Set(nodes=mesh.MeshNodeArray(self.node_list), name="Accelometers")

    def create_history_output(self):
        self.soilModel.fieldOutputRequests["F-Output-1"].deactivate("Vibration Step")
        self.soilModel.HistoryOutputRequest(createStepName="Vibration Step", frequency=1, name="H-Output-2",
                                            variables=('A2','V2'),
                                            region=self.soilModel.rootAssembly.allInstances['Soil Part-1'].sets[
                                                'Accelometers'])

        del self.soilModel.historyOutputRequests['H-Output-1']
        del self.soilModel.fieldOutputRequests['F-Output-1']

    def create_job(self):
        self.job_name = self.model_name
        mdb.Job(model=self.model_name, name=self.job_name, type=ANALYSIS, memory=90, memoryUnits=PERCENTAGE)
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
        for i in range(len(self.parameter["Name"])):
            name = self.parameter["Name"][i]
            height = sum(self.parameter["Thicknesses"][:i + 1])
            density = self.parameter["Density"][i]
            thickness = self.parameter["Thicknesses"][i]
            Vs = self.parameter["Vs"][i]
            elasticity = (self.parameter["Elastic"][i], self.parameter["Poisson"][i])
            damping_ratio = self.parameter["Damping Ratio"]

            self.partition_part(height)
            self.create_material(name, {"E": elasticity, "D": density, "Damping": damping_ratio, "Vs": Vs,"H": thickness})

        self.create_section()
        self.create_edge_set()
        self.draw_source()
        self.create_instance()
        self.create_step()
        self.create_face_sets()
        self.create_sheet_pile()
        self.create_rubber_barrier()
        self.create_ditch()
        self.create_mesh()
        self.create_nodes()
        self.create_boundary_conditions()
        self.create_history_output()
        self.create_job()
        self.change_element_type()

def read_json(file_name)->Input:
    with open(file_name) as f:
        data = json.load(f)
    
    return Input(**data)

model = Create_Model()
model.operator()
