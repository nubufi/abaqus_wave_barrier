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

class Create_Model:
    def __init__(self):
        self.parameter = {"Name": temp_layer_names,
                          "Elastic": temp_elastic,
                          "Poisson": temp_poisson,
                          "Density": temp_density,
                          "Thicknesses": temp_thickness,
                          "Damping Ratio": temp_damping_ratio,
                          "Vs": temp_VS}

        self.inf_size_x = 10
        self.inf_size_y = 10
        self.Height = float64(sum(self.parameter["Thicknesses"]))+self.inf_size_y
        self.source_size = float64(temp_source_size)
        self.source_coordinate = self.inf_size_x + 1 + self.source_size
        self.accelometer_pattern = temp_accelerometer_pattern
        self.Width = self.accelometer_pattern[-1]+self.source_coordinate + 1 + self.inf_size_x
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

        self.SP_pattern = temp_SP_pattern
        self.SP_E = temp_SP_E
        self.SP_density = temp_SP_density
        self.SP_thickness = temp_SP_thickness
        self.SP_height = temp_SP_height
        self.SP_interaction = temp_SP_interaction

        self.Thicknesses = self.parameter["Thicknesses"]
        self.layer_heights = np.append([self.Height], self.Height - cumsum(self.parameter["Thicknesses"]))
        self.sorted_heights = array(sorted(self.layer_heights)[1:])

        self.model_name = "temp_model_name"

        self.mesh_size = temp_mesh_size
        self.f = lambda L, x: [L[i:i + 1] for i in x]
        session.viewports["Viewport: 1"].setValues(displayedObject=None)
        mdb.models.changeKey(fromName="Model-1", toName=self.model_name)
        self.soilModel = mdb.models[self.model_name]

    def create_part(self):
        soilProfileSketch = self.soilModel.ConstrainedSketch(
            name="__profile__", sheetSize=self.Width * 2)
        soilProfileSketch.sketchOptions.setValues(viewStyle=AXISYM)
        soilProfileSketch.ConstructionLine(
            point1=(0.0, -100.0), point2=(0.0, 100.0))
        soilProfileSketch.FixedConstraint(entity=soilProfileSketch.geometry[2])
        soilProfileSketch.rectangle(
            point1=(0, 0), point2=(self.Width, self.Height))
        soilProfileSketch.CoincidentConstraint(addUndoState=False, entity1=soilProfileSketch.vertices[0],
                                               entity2=soilProfileSketch.geometry[2])
        self.soilPart = self.soilModel.Part(
            name="Soil Part", dimensionality=TWO_D_PLANAR, type=DEFORMABLE_BODY)
        self.soilPart.BaseShell(sketch=soilProfileSketch)
        self.soilPart.Set(faces=self.f(
            self.soilPart.faces, [0]), name="All Face")
        del self.soilModel.sketches['__profile__']

        self.soilModel.ConstrainedSketch(gridSpacing=3.53, name='__profile__', sheetSize=self.Width * 2,
                                         transform=self.soilPart.MakeSketchTransform(
                                             sketchPlane=self.soilPart.faces[0],
                                             sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(0, 0, 0.0)))

        self.soilPart.projectReferencesOntoSketch(filter=COPLANAR_EDGES,
                                                  sketch=self.soilModel.sketches['__profile__'])
        self.soilModel.sketches['__profile__'].rectangle(point1=(self.inf_size_x, self.Height),
                                                         point2=(self.Width - self.inf_size_x, self.inf_size_y))

        self.soilModel.sketches['__profile__'].Line(point1=(self.Width - self.inf_size_x, self.inf_size_y),
                                                    point2=(self.Width, 0))

        self.soilModel.sketches['__profile__'].Line(point1=(self.inf_size_x, self.inf_size_y),
                                                    point2=(0, 0))

        self.soilModel.sketches['__profile__'].Line(
            point1=(self.Width - self.inf_size_x -
                    self.mesh_size, self.inf_size_y),
            point2=(self.Width - self.inf_size_x - self.mesh_size, 0))

        self.soilModel.sketches['__profile__'].Line(
            point1=(self.Width - self.inf_size_x,
                    self.inf_size_y + self.mesh_size),
            point2=(self.Width, self.inf_size_y + self.mesh_size))

        self.soilModel.sketches['__profile__'].Line(
            point1=(self.inf_size_x + self.mesh_size, self.inf_size_y),
            point2=(self.inf_size_x + self.mesh_size, 0))

        self.soilModel.sketches['__profile__'].Line(
            point1=(0, self.inf_size_y + self.mesh_size),
            point2=(self.inf_size_x, self.inf_size_y + self.mesh_size))

        self.soilPart.PartitionFaceBySketch(faces=self.soilPart.faces[0],
                                            sketch=self.soilModel.sketches['__profile__'])
        del self.soilModel.sketches['__profile__']

    def sketch_face(self, y, left_x):
        face = self.soilPart.faces.findAt((0.01 + left_x, y - 0.01, 0))
        self.soilModel.ConstrainedSketch(gridSpacing=3.53, name='__profile__', sheetSize=self.Width,
                                         transform=self.soilPart.MakeSketchTransform(
                                             sketchPlane=face,
                                             sketchPlaneSide=SIDE1,
                                             sketchOrientation=RIGHT,
                                             origin=(0, 0, 0.0)))
        self.soilPart.projectReferencesOntoSketch(filter=COPLANAR_EDGES,
                                                  sketch=self.soilModel.sketches['__profile__'])
        self.soilModel.sketches['__profile__'].Line(point1=(0 + left_x, y),
                                                    point2=(self.Width - self.inf_size_x + left_x, y))
        self.soilPart.PartitionFaceBySketch(
            faces=face, sketch=self.soilModel.sketches['__profile__'])
        del self.soilModel.sketches['__profile__']

    def partition_part(self, thickness):
        offset = self.Height - thickness
        if offset > self.inf_size_y:
            self.sketch_face(offset, 0)
            self.sketch_face(offset, self.inf_size_x)
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
        inf_Bottom_Nodes = [
            (self.Width*0.5, 0, 0),
            (0, self.inf_size_y+self.mesh_size+0.01, 0),
            (0, self.inf_size_y, 0),
            (self.inf_size_x, 0, 0),
            (self.Width - self.inf_size_x, 0, 0),
            (self.Width-0.01, self.inf_size_y, 0),
            (self.Width-0.01, self.inf_size_y+self.mesh_size+0.01, 0),
        ]
        inf_bottom_faces = []
        for node in inf_Bottom_Nodes:
            inf_bottom_faces.append(
                self.soilPart.faces[self.soilPart.faces.findAt(node).index])

        self.soilModel.HomogeneousSolidSection(
            name="Infinite_Section", material=self.parameter["Name"][-1])
        self.soilPart.SectionAssignment(
            region=inf_bottom_faces, sectionName="Infinite_Section")

        for i in range(len(self.layer_heights) - 1):
            y = float64(self.layer_heights[i]) - 0.01
            name = self.parameter["Name"][i]

            index1 = self.soilPart.faces.findAt((0.01, y, 0)).index
            index2 = self.soilPart.faces.findAt(
                (self.Width - 0.01, y, 0)).index
            index3 = self.soilPart.faces.findAt((self.Width*0.5, y, 0)).index
            section_name = name + "_Section"

            self.soilModel.HomogeneousSolidSection(
                name=section_name, material=name)
            self.soilPart.SectionAssignment(region=(
                self.soilPart.faces[index1],
                self.soilPart.faces[index2],
                self.soilPart.faces[index3]),
                sectionName=section_name)

    def create_edge_set(self):
        left_finite_edges = []
        finite_horizontal_edges = []
        vertical_edges = []
        single_seed_edges = []
        single_seed_coordinates = [
            (0.1, self.Height, 0),
            (0.1, self.inf_size_y+self.mesh_size, 0),
            (0, 0.01, 0),
            (1, 1, 0),
            (1, 0, 0),
            (self.inf_size_x+self.mesh_size, 1, 0),
            (self.inf_size_x+self.mesh_size, 0.1, 0),
            (self.Width - self.inf_size_x - self.mesh_size, 0.01, 0),
            (self.Width - 0.01, self.Height, 0),
            (self.Width - 0.01, self.inf_size_y + self.mesh_size, 0),
            (self.Width - 0.01, 0, 0), (self.Width, 0.01, 0),
            ((self.Width - self.inf_size_x / 2), self.inf_size_y / 2, 0)
        ]
        for i in single_seed_coordinates:
            single_seed_edges.append(self.soilPart.edges.findAt(i).index)

        y_coordinates = list(self.layer_heights)
        for y in y_coordinates:
            finite_edge_id = self.soilPart.edges.findAt(
                (self.inf_size_x+1, y, 0)).index
            finite_horizontal_edges.append(finite_edge_id)

        for y in arange(self.Height-0.01, self.inf_size_y + self.mesh_size, -0.1):
            # Vertical Edges
            N1 = (0, y, 0)
            N2 = (self.Width - self.inf_size_x, y, 0)
            N3 = (self.Width, y, 0)
            N4 = (self.inf_size_x, y, 0)
            try:
                id1 = self.soilPart.edges.findAt(N1).index
                id2 = self.soilPart.edges.findAt(N2).index
                id3 = self.soilPart.edges.findAt(N3).index
                id4 = self.soilPart.edges.findAt(N4).index

                left_finite_edges.append(id1)
                vertical_edges.append(id1)
                vertical_edges.append(id2)
                vertical_edges.append(id3)
                vertical_edges.append(id4)
            except:
                pass

        self.soilPart.Set(edges=self.f(self.soilPart.edges,
                                       left_finite_edges), name="Left_Vertical_Edges")
        self.soilPart.Set(edges=self.f(self.soilPart.edges,
                                       vertical_edges), name="All_Vertical_Edges")
        self.soilPart.Set(edges=self.f(self.soilPart.edges, finite_horizontal_edges),
                          name="Finite_Horizontal_Edges")
        self.soilPart.Set(edges=self.f(self.soilPart.edges,
                                       single_seed_edges), name="Single_Seed_Edges")

    def draw_source(self):
        N1 = (self.source_coordinate - self.source_size, self.Height, 0)
        N2 = (self.source_coordinate, self.Height, 0)

        self.soilPart.DatumPointByCoordinate(N1)
        self.soilPart.DatumPointByCoordinate(N2)

        top_part = self.soilPart.faces.findAt(N1)
        self.soilPart.PartitionFaceByShortestPath(
            faces=top_part, point1=N1, point2=N2)
        id1 = self.soilPart.edges.findAt(
            (0.5 * self.Width, self.Height, 0)).index
        self.soilPart.Set(edges=self.f(
            self.soilPart.edges, [id1]), name="Source")

    def create_instance(self):
        self.soilModel.rootAssembly.DatumCsysByDefault(CARTESIAN)
        self.soilModel.rootAssembly.Instance(
            dependent=ON, name="Soil Part-1", part=self.soilPart)
        self.soilModel.rootAssembly.regenerate()

    def create_step(self):
        self.create_vibration()
        self.soilModel.ImplicitDynamicsStep(initialInc=self.time_step, timePeriod=self.duration,
                                            maxNumInc=int(
                                                2 * (self.duration / self.time_step)),
                                            name='Vibration Step',
                                            previous='Initial', maxInc=self.time_step)
        self.soilModel.TabularAmplitude(
            name="Vibration", timeSpan=STEP, smooth=SOLVER_DEFAULT, data=self.data)

    def create_face_sets(self):
        finite_faces = self.soilPart.faces.getByBoundingBox(
            self.inf_size_x, self.inf_size_y, 0, self.Width - self.inf_size_x, self.Height, 0)
        self.soilPart.Set(faces=finite_faces, name="Finite_Faces")

        inf_face1 = self.soilPart.faces.findAt(
            (0.01, self.inf_size_y, 0)).index
        inf_face2 = self.soilPart.faces.findAt(
            (self.inf_size_x, 0.01, 0)).index
        inf_face3 = self.soilPart.faces.findAt((self.Width * 0.5, 1, 0)).index
        inf_face4 = self.soilPart.faces.findAt(
            (self.Width-0.1, self.inf_size_y, 0)).index
        inf_face5 = self.soilPart.faces.findAt(
            (self.Width-self.inf_size_x, 0.01, 0)).index
        infinite_faces = [inf_face1, inf_face2,
                          inf_face3, inf_face4, inf_face5]

        for i in range(len(self.layer_heights)):
            y = float64(self.layer_heights[i] - 0.01)
            inf_face_left = self.soilPart.faces.findAt((0.01, y, 0)).index
            inf_face_right = self.soilPart.faces.findAt(
                (self.Width - 0.01, y, 0)).index
            infinite_faces.append(inf_face_left)
            infinite_faces.append(inf_face_right)

        self.soilPart.Set(faces=self.f(self.soilPart.faces,
                                       infinite_faces), name="Infinite_Faces")

    def create_sheet_pile(self):
        if self.SP_pattern:
            # Cut Soil
            face = self.soilPart.sets["All Face"].faces[0]
            self.soilModel.ConstrainedSketch(gridSpacing=3.53, name='__profile__', sheetSize=self.Width,
                                             transform=self.soilPart.MakeSketchTransform(sketchPlane=face,
                                                                                         sketchPlaneSide=SIDE1,
                                                                                         sketchOrientation=RIGHT,
                                                                                         origin=(0, 0, 0.0)))
            self.soilPart.projectReferencesOntoSketch(filter=COPLANAR_EDGES,
                                                      sketch=self.soilModel.sketches['__profile__'])
            self.soilModel.sketches['__profile__'].rectangle(
                point1=(self.source_coordinate+self.SP_pattern, self.Height),
                point2=(self.source_coordinate+self.SP_pattern+self.SP_thickness, self.Height - self.SP_height))
            self.soilPart.Cut(
                sketch=self.soilModel.sketches['__profile__'])
            del self.soilModel.sketches['__profile__']

            # Create Part
            SPSketch = self.soilModel.ConstrainedSketch(
                name="__profile__", sheetSize=self.Width * 2)
            SPSketch.sketchOptions.setValues(viewStyle=AXISYM)
            SPSketch.ConstructionLine(
                point1=(0.0, -100.0), point2=(0.0, 100.0))
            SPSketch.FixedConstraint(entity=SPSketch.geometry[2])
            SPSketch.rectangle(point1=(0, 0), point2=(
                self.SP_thickness, self.SP_height))
            SPSketch.CoincidentConstraint(addUndoState=False, entity1=SPSketch.vertices[0],
                                          entity2=SPSketch.geometry[2])
            self.SPPart = self.soilModel.Part(
                name="Sheet Pile", dimensionality=TWO_D_PLANAR, type=DEFORMABLE_BODY)
            self.SPPart.BaseShell(sketch=SPSketch)
            SPFace = self.SPPart.faces
            self.SPPart.Set(faces=self.f(SPFace, [0]), name="Sheet Pile")
            del self.soilModel.sketches['__profile__']

            # Section Assignment
            self.soilMaterial = self.soilModel.Material("Steel")
            self.soilMaterial.Density(table=((self.SP_density,),))
            self.soilMaterial.Elastic(table=((self.SP_E, 0.33),))
            self.soilModel.HomogeneousSolidSection(
                name="Sheet Pile Section", material="Steel")
            self.SPPart.SectionAssignment(
                region=(SPFace[0],), sectionName="Sheet Pile Section")

            # Create Instance
            self.soilModel.rootAssembly.DatumCsysByDefault(CARTESIAN)
            self.soilModel.rootAssembly.Instance(
                dependent=ON, name="Sheet Pile-1", part=self.SPPart)
            self.soilModel.rootAssembly.regenerate()
            self.soilModel.rootAssembly.translate(instanceList=('Sheet Pile-1', ),
                                                  vector=(self.source_coordinate+self.SP_pattern, self.Height-self.SP_height, 0))

            # Create Interaction
            self.soilModel.ContactProperty('IntProp-1')
            self.soilModel.interactionProperties['IntProp-1'].NormalBehavior(
                allowSeparation=ON, constraintEnforcementMethod=DEFAULT, pressureOverclosure=HARD)
            self.soilModel.ContactStd(
                createStepName='Initial', name='Int-1')
            self.soilModel.interactions['Int-1'].includedPairs.setValuesInStep(
                stepName='Initial', useAllstar=ON)
            self.soilModel.interactions['Int-1'].contactPropertyAssignments.appendInStep(
                assignments=((GLOBAL, SELF, 'IntProp-1'), ), stepName='Initial')
            
            #Create Mesh
            self.SPPart.seedPart(size=self.mesh_size)
            self.SPPart.generateMesh()

    def create_rubber_barrier(self):
        if self.fillDitch and self.ditchnumber:
            # Create Part
            RCSkectch = self.soilModel.ConstrainedSketch(
                name="__profile__", sheetSize=self.Width * 2)
            RCSkectch.sketchOptions.setValues(viewStyle=AXISYM)
            RCSkectch.ConstructionLine(
                point1=(0.0, -100.0), point2=(0.0, 100.0))
            RCSkectch.FixedConstraint(entity=RCSkectch.geometry[2])
            RCSkectch.rectangle(point1=(0, 0), point2=(
                self.ditch_width, self.ditch_depth))
            RCSkectch.CoincidentConstraint(addUndoState=False, entity1=RCSkectch.vertices[0],
                                          entity2=RCSkectch.geometry[2])
            self.RCPart = self.soilModel.Part(
                name="Rubber Chip", dimensionality=TWO_D_PLANAR, type=DEFORMABLE_BODY)
            self.RCPart.BaseShell(sketch=RCSkectch)
            RCFace = self.RCPart.faces
            self.RCPart.Set(faces=self.f(RCFace, [0]), name="Rubber Chip")
            del self.soilModel.sketches['__profile__']

            # Section Assignment
            alpha, beta = self.rayleigh_damping(
                [1, 4], self.RC_VS, self.ditch_depth, self.RC_damping)
            self.soilMaterial = self.soilModel.Material("Rubber")
            self.soilMaterial.Density(table=((self.RC_density,),))
            self.soilMaterial.Elastic(table=((self.RC_E, 0.25),))
            self.soilMaterial.Damping(alpha=alpha, beta=beta)
            self.soilModel.HomogeneousSolidSection(
                name="Rubber Chip Section", material="Rubber")
            self.RCPart.SectionAssignment(
                region=(RCFace[0],), sectionName="Rubber Chip Section")

            # Create Instance
            self.soilModel.rootAssembly.DatumCsysByDefault(CARTESIAN)
            self.soilModel.rootAssembly.Instance(
                dependent=ON, name="Rubber Chip-1", part=self.RCPart)
            self.soilModel.rootAssembly.regenerate()
            self.soilModel.rootAssembly.translate(
                instanceList=('Rubber Chip-1', ),
                vector=(self.source_coordinate+self.ditch2source, self.Height-self.ditch_depth, 0))

            # Create Interaction
            try:
                self.soilModel.ContactProperty('IntProp-1')
                self.soilModel.interactionProperties['IntProp-1'].NormalBehavior(
                    allowSeparation=ON, constraintEnforcementMethod=DEFAULT, pressureOverclosure=HARD)
                self.soilModel.ContactStd(
                    createStepName='Initial', name='Int-1')
                self.soilModel.interactions['Int-1'].includedPairs.setValuesInStep(
                    stepName='Initial', useAllstar=ON)
                self.soilModel.interactions['Int-1'].contactPropertyAssignments.appendInStep(
                    assignments=((GLOBAL, SELF, 'IntProp-1'), ), stepName='Initial')
            except:
                pass
            
            #Create Mesh
            self.RCPart.seedPart(size=self.mesh_size)
            self.RCPart.generateMesh()

            
    def create_ditch(self):
        if self.ditchnumber > 0:
            top_face = self.soilPart.faces.findAt((self.Width*0.5, self.Height - 0.01, 0))
            for i in range(self.ditchnumber):
                x = self.source_coordinate + self.ditch2source + \
                    i*(self.ditch_width + self.ditch2ditch)
                self.soilModel.ConstrainedSketch(gridSpacing=3.53, name='__profile__', sheetSize=self.Width,
                                                 transform=self.soilPart.MakeSketchTransform(sketchPlane=top_face,
                                                                                             sketchPlaneSide=SIDE1,
                                                                                             sketchOrientation=RIGHT,
                                                                                             origin=(0, 0, 0.0)))

                self.soilPart.projectReferencesOntoSketch(
                    filter=COPLANAR_EDGES, sketch=self.soilModel.sketches['__profile__'])
                self.soilModel.sketches['__profile__'].rectangle(point1=(x, self.Height),
                                                                 point2=(x + self.ditch_width, self.Height - self.ditch_depth))

                self.soilPart.Cut(sketch=self.soilModel.sketches['__profile__'])
                del self.soilModel.sketches['__profile__']

            """try:
                for x in [0.01]+list(self.SP_pattern+self.SP_thickness+0.01)+[self.Width-0.01]:
                    face = self.soilPart.faces.findAt(
                        (x, self.Height - self.ditch_depth, 0))
                    self.soilModel.ConstrainedSketch(gridSpacing=3.53, name='__profile__', sheetSize=self.Width,
                                                     transform=self.soilPart.MakeSketchTransform(sketchPlane=face,
                                                                                                 sketchPlaneSide=SIDE1,
                                                                                                 sketchOrientation=RIGHT,
                                                                                                 origin=(0, 0, 0.0)))
                    self.soilPart.projectReferencesOntoSketch(
                        filter=COPLANAR_EDGES, sketch=self.soilModel.sketches['__profile__'])
                    self.soilModel.sketches['__profile__'].Line(point1=(0, self.Height - self.ditch_depth),
                                                                point2=(self.Width, self.Height - self.ditch_depth))
                    self.soilPart.PartitionFaceBySketch(
                        faces=face, sketch=self.soilModel.sketches['__profile__'])
                    del self.soilModel.sketches['__profile__']
            except:
                pass"""

    def set_mesh_control(self):
        face_RB = self.soilPart.faces.findAt((self.Width - self.inf_size_x,0.1 , 0))
        face_RT = self.soilPart.faces.findAt((self.Width - 0.1, self.inf_size_y, 0))
        diagonal_edge_right = self.soilPart.edges.findAt(
            (self.Width - self.inf_size_x/2, self.inf_size_y/2, 0))

        face_LB = self.soilPart.faces.findAt((self.inf_size_x,0.1 , 0))
        face_LT = self.soilPart.faces.findAt((0.1, self.inf_size_y, 0))
        diagonal_edge_left = self.soilPart.edges.findAt(
            (self.inf_size_x/2, self.inf_size_y/2, 0))
        
        face_bottom = self.soilPart.faces.findAt((self.Width*0.5,0.1 , 0))
        vertical_edge = self.soilPart.edges.findAt(
            (self.inf_size_x+self.mesh_size, self.inf_size_y/2, 0))

        self.soilPart.setSweepPath(
            edge=diagonal_edge_left, region=face_LB, sense=FORWARD)
        self.soilPart.setSweepPath(
            edge=diagonal_edge_left, region=face_LT, sense=FORWARD)
        self.soilPart.setSweepPath(
            edge=diagonal_edge_right, region=face_RB, sense=FORWARD)
        self.soilPart.setSweepPath(
            edge=diagonal_edge_right, region=face_RT, sense=FORWARD)
        self.soilPart.setSweepPath(
            edge=vertical_edge, region=face_bottom, sense=FORWARD)
        self.soilPart.setMeshControls(
            regions=self.soilPart.sets["Infinite_Faces"].faces, technique=SWEEP)

        for y in self.layer_heights[:-1]:
            face_left = self.soilPart.faces.findAt((0.1, y-0.01, 0))
            edge_left = self.soilPart.edges.findAt((0.1, y, 0))
            self.soilPart.setSweepPath(edge=edge_left, region=face_left, sense=REVERSE)

            face_right = self.soilPart.faces.findAt((self.Width - 0.1, y-0.01, 0))
            edge_right = self.soilPart.edges.findAt((self.Width - 0.1, y, 0))
            self.soilPart.setSweepPath(edge=edge_right, region=face_right, sense=FORWARD)

        try:
            if self.Height - self.ditch_depth not in self.layer_heights:
                y = self.Height - self.ditch_depth
                face = self.soilPart.faces.findAt(
                    (self.Width - 0.01, y-0.01, 0))
                edge = self.soilPart.edges.findAt((self.Width-0.01, y, 0))
                self.soilPart.setSweepPath(
                    edge=edge, region=face, sense=FORWARD)
        except:
            pass

    def create_mesh(self):
        self.set_mesh_control()
        self.soilPart.setElementType(regions=(self.soilPart.sets["Infinite_Faces"]), elemTypes=(
            mesh.ElemType(elemCode=CPE4I, elemLibrary=STANDARD), mesh.ElemType(elemCode=CPE4I, elemLibrary=STANDARD)))
        self.soilPart.setElementType(regions=(self.soilPart.sets["Finite_Faces"]), elemTypes=(
            mesh.ElemType(elemCode=CPE4, elemLibrary=STANDARD), mesh.ElemType(elemCode=CPE4, elemLibrary=STANDARD)))
        self.soilPart.seedEdgeBySize(edges=self.soilPart.sets["All_Vertical_Edges"].edges, size=self.mesh_size,
                                     constraint=FIXED)
        self.soilPart.seedEdgeBySize(edges=self.soilPart.sets["Finite_Horizontal_Edges"].edges, size=self.mesh_size,
                                     constraint=FIXED)
        self.soilPart.seedEdgeBySize(
            edges=(self.soilPart.edges.findAt((self.Width*0.5, 0, 0)),), size=self.mesh_size)
        self.soilPart.seedEdgeByNumber(edges=self.soilPart.sets["Single_Seed_Edges"].edges, number=1,
                                       constraint=FIXED)

        self.soilPart.generateMesh()
    
    def create_nodes(self):
        self.node_list = []
        x2 = self.source_coordinate + array(self.accelometer_pattern)
        for i in x2:
            self.node_list.append(
                self.soilPart.nodes.getClosest((i, self.Height, 0)))
            self.node_list.append(
                self.soilPart.nodes.getClosest((i, self.Height, 0)))

        self.soilPart.Set(nodes=mesh.MeshNodeArray(
            self.node_list), name="Accelometers")

    def create_boundary_conditions(self):
        #Sets = self.soilModel.rootAssembly.instances["Soil Part-1"].sets

        #source_edge = self.soilPart.edges.findAt((self.source_coordinate-0.1, self.Height, 0)).index
        self.soilPart.Set(nodes=mesh.MeshNodeArray(
            [self.soilPart.nodes.getClosest((self.source_coordinate,self.Height,0))]), 
            name="LoadPoint")
        #self.soilPart.Surface(name="Source_Top", side1Edges=self.f(self.soilPart.edges, [source_edge, ]))
        #self.soilModel.Pressure(amplitude='Vibration', createStepName='Vibration Step', magnitude=self.PGA,name='Vibration',
        #                        region=self.soilModel.rootAssembly.instances['Soil Part-1'].surfaces['Source_Top'])
        self.soilModel.ConcentratedForce(amplitude='Vibration', cf2= -1000.0, createStepName='Vibration Step', 
                distributionType=UNIFORM, field='',localCsys=None, name='Vibration', 
                region=self.soilModel.rootAssembly.instances['Soil Part-1'].sets['LoadPoint'])

    def create_history_output(self):
        self.soilModel.fieldOutputRequests["F-Output-1"].deactivate(
            "Vibration Step")
        self.soilModel.HistoryOutputRequest(createStepName="Vibration Step", frequency=1, name="H-Output-2",
                                            variables=('A2', 'V2'),
                                            region=self.soilModel.rootAssembly.allInstances['Soil Part-1'].sets[
                                                'Accelometers'])

        del self.soilModel.historyOutputRequests['H-Output-1']
        del self.soilModel.fieldOutputRequests['F-Output-1']

    def create_job(self):
        self.job_name = self.model_name
        mdb.Job(model=self.model_name, name=self.job_name,
                type=ANALYSIS, memory=90, memoryUnits=PERCENTAGE)
        mdb.jobs[self.model_name].writeInput(consistencyChecking=OFF)

    def change_element_type(self):
        inp = open(self.model_name + ".inp")
        data = inp.read().replace("CPE4I", "CINPE4")
        new_inp = open(self.model_name + ".inp", "w")
        new_inp.write(data)
        new_inp.close()
        inp.close()
    
    def rayleigh_damping(self, modes, Vs, H, damping_ratio):
        def wn(mode, Vs, H): return (2 * mode - 1) * Vs / (4 * H)
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

    def operator(self):
        self.create_part()
        for i in range(len(self.parameter["Name"])):
            name = self.parameter["Name"][i]
            height = sum(self.parameter["Thicknesses"][:i + 1])
            density = self.parameter["Density"][i]
            thickness = self.parameter["Thicknesses"][i]
            Vs = self.parameter["Vs"][i]
            elasticity = (self.parameter["Elastic"]
                          [i], self.parameter["Poisson"][i])
            damping_ratio = self.parameter["Damping Ratio"]

            self.partition_part(height)
            self.create_material(name, {
                                 "E": elasticity, "D": density, "Damping": damping_ratio, "Vs": Vs, "H": thickness})

        self.create_section()
        self.create_edge_set()
        self.draw_source()
        self.create_instance()
        self.create_step()
        self.create_face_sets()
        self.create_sheet_pile()
        self.create_ditch()
        self.create_rubber_barrier()
        self.create_mesh()
        self.create_nodes()
        self.create_boundary_conditions()
        self.create_history_output()
        self.create_job()
        self.change_element_type()

model = Create_Model()
model.operator()
