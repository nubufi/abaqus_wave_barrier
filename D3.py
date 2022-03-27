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
from numpy import float, cumsum, array, arange, pi, flipud
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

        self.inf_size_x = 5
        self.inf_size_y = 5
        self.inf_size_z = 1
        self.Width = float(temp_width)+2*self.inf_size_x
        self.Depth = float(sum(self.parameter["Thicknesses"]))+self.inf_size_z
        self.Length = self.Width
        self.source_size = float(temp_source_size)

        self.layer_heights = np.append([self.Depth], self.Depth - cumsum(self.parameter["Thicknesses"]))
        self.sorted_heights = array(sorted(self.layer_heights)[1:])
        self.accelometer_pattern = temp_accelerometer_pattern

        self.PGA = temp_PGA
        self.duration = temp_duration
        self.frequency = temp_frequency
        self.time_step = temp_time_step

        self.ditch_width = temp_ditch_width
        self.ditch_height = temp_ditch_depth
        self.ditch_length = temp_ditch_length
        self.ditch2source = temp_ditch2source
        self.ditch_number = temp_ditchnumber
        self.ditch2ditch = temp_ditch2ditch

        self.fill_ditch = int(temp_fill_ditch)
        self.RC_E = temp_RC_E
        self.RC_density = temp_RC_density
        self.RC_damping = temp_RC_damping
        self.RC_VS = temp_RC_VS


        self.free_space_X = float((self.Width - self.source_size))*0.5
        self.free_space_Y = float(self.inf_size_y + 1)

        self.SP_pattern = array([temp_SP_pattern])+self.source_size + self.free_space_Y
        self.SP_E = temp_SP_E
        self.SP_density = temp_SP_density
        self.SP_thickness = temp_SP_thickness
        self.SP_height = temp_SP_height
        self.SP_interaction = temp_SP_interaction

        self.Thicknesses = self.parameter["Thicknesses"]

        self.model_size = sum(self.accelometer_pattern) + self.source_size
        self.model_name = "temp_model_name"

        self.finite_mesh_type = "C3D8R"
        self.infinite_mesh_type = "CIN3D8"
        self.temp_inf_type = "AC3D8"

        self.mesh_size = temp_mesh_size
        self.f = lambda L, x: [L[i:i + 1] for i in x]
        session.viewports["Viewport: 1"].setValues(displayedObject=None)
        mdb.models.changeKey(fromName="Model-1", toName=self.model_name)
        self.soilModel = mdb.models[self.model_name]

    def layer_centers(self):
        self.centers = []
        layer_heights = cumsum(flipud(self.parameter["Thicknesses"]))
        for i in range(len(layer_heights)):
            if i == 0:
                center = layer_heights[i] / 2
            else:
                center = layer_heights[i - 1] + (layer_heights[i] - layer_heights[i - 1]) / 2
            self.centers.append(center)

    def create_part(self):
        #Create Part
        soilProfileSketch = self.soilModel.ConstrainedSketch(name="__profile__", sheetSize=self.Width * 2)
        x1 = self.inf_size_x
        x2 = self.Width - self.inf_size_x
        x3 = self.Width
        y1 = self.inf_size_y
        y2 = self.Length - self.inf_size_y
        y3 = self.Length

        soilProfileSketch.Line(point1=(x1,0),point2=(x2,0))
        soilProfileSketch.Line(point1=(x2,0),point2=(x2,y1))
        soilProfileSketch.Line(point1=(x2,y1),point2=(x3,y1))
        soilProfileSketch.Line(point1=(x3,y1),point2=(x3,y2))
        soilProfileSketch.Line(point1=(x3,y2),point2=(x2,y2))
        soilProfileSketch.Line(point1=(x2,y2),point2=(x2,y3))
        soilProfileSketch.Line(point1=(x2,y3),point2=(x1,y3))
        soilProfileSketch.Line(point1=(x1,y3),point2=(x1,y2))
        soilProfileSketch.Line(point1=(x1,y2),point2=(0,y2))
        soilProfileSketch.Line(point1=(0,y2),point2=(0,y1))
        soilProfileSketch.Line(point1=(0,y1),point2=(x1,y1))
        soilProfileSketch.Line(point1=(x1,y1),point2=(x1,0))


        self.soilPart = self.soilModel.Part(name="Soil Part", dimensionality=THREE_D, type=DEFORMABLE_BODY)
        self.soilPart.BaseSolidExtrude(sketch=soilProfileSketch, depth=self.Depth)

        del self.soilModel.sketches['__profile__']

        self.create_face_sets()

        #Draw Infinite Lines
        self.soilModel.ConstrainedSketch(gridSpacing=2.37, name='__profile__',
                                        sheetSize=self.Width, transform=self.soilPart.MakeSketchTransform(
                                         sketchPlane=self.soilPart.faces.findAt((self.Width*0.5,self.Length*0.5,self.Depth)),
                                         sketchPlaneSide=SIDE1,sketchOrientation=RIGHT, origin=(0, 0, self.Depth),
                                         sketchUpEdge=self.soilPart.edges.findAt((self.Width,self.Length*0.5,self.Depth))))

        self.x_border_Left = self.inf_size_x
        self.x_border_Right = self.Width - self.inf_size_x
        self.y_border_Top = self.Length - self.inf_size_y
        self.y_border_Bottom = self.inf_size_y

        self.soilPart.projectReferencesOntoSketch(filter=COPLANAR_EDGES,sketch=self.soilModel.sketches['__profile__'])
        self.soilModel.sketches['__profile__'].rectangle(point1=(self.x_border_Left, self.y_border_Bottom),point2=(self.x_border_Right, self.y_border_Top))

        self.soilPart.PartitionFaceBySketch(faces=self.soilPart.faces.findAt((self.Width*0.5,self.Length*0.5,self.Depth)),
                                            sketch=self.soilModel.sketches['__profile__'],
                                            sketchUpEdge=self.soilPart.edges.findAt((self.Width,self.Length*0.5,self.Depth)))

        self.soilPart.PartitionCellBySweepEdge(cells=self.soilPart.cells,edges=(
                                               self.soilPart.edges.findAt((self.x_border_Left,self.Length*0.5,self.Depth)),
                                               self.soilPart.edges.findAt((self.x_border_Right,self.Length*0.5,self.Depth)),
                                               self.soilPart.edges.findAt((self.Width*0.5,self.y_border_Bottom,self.Depth)),
                                               self.soilPart.edges.findAt((self.Width*0.5,self.y_border_Top,self.Depth))),
                                               sweepPath=self.soilPart.edges.findAt((0,self.inf_size_y,self.Depth*0.5)))

        del self.soilModel.sketches['__profile__']

    def partition_cell(self, z):
        try:
            plane = self.soilPart.DatumPlaneByPrincipalPlane(offset=z, principalPlane=XYPLANE).id
            self.soilPart.PartitionCellByDatumPlane(cells=self.soilPart.cells, datumPlane=self.soilPart.datums[plane])
        except:
            pass

    def natural_frequency(self, mode, Vs, H):
        f = (2 * mode - 1) * Vs / (4 * H)
        return f

    def rayleigh_damping(self, modes, Vs, H, damping_ratio):
        if Vs == 0:
            return 0, 0
        else:
            wi = self.natural_frequency(modes[0], Vs, H)
            wj = self.natural_frequency(modes[1], Vs, H)
            alpha = damping_ratio * 2 * wi * wj / (wi + wj)
            beta = damping_ratio * 2 / (wi + wj)

            return alpha, beta

    def create_material(self, name, properties):
        elastic = properties["E"]
        density = properties["D"]
        damping = properties.get("Damping",0)
        H = properties.get("H", 0.005)
        Vs = properties.get("Vs", 0)
        self.soilMaterial = self.soilModel.Material(name)

        if Vs > 0:
            alpha, beta = self.rayleigh_damping([1, 4], Vs, H, damping)
            self.soilMaterial.Damping(alpha=alpha, beta=beta)

        #alpha, beta = float(5.89),float(0.0003979)
        self.soilMaterial.Density(table=((density,),))
        self.soilMaterial.Elastic(table=(elastic,))

    def create_section(self):
        for i in range(len(self.centers)):
            z = float(sorted(self.centers,reverse=True)[i]) + self.inf_size_z
            name = self.parameter["Name"][i]

            index1 = self.soilPart.cells.findAt((self.Width*0.5, self.Length*0.5, z)).index
            print(z)
            index2 = self.soilPart.cells.findAt((self.Width*0.5, 0.1, z)).index
            index3 = self.soilPart.cells.findAt((self.Width - 0.1, self.Length*0.5, z)).index
            index4 = self.soilPart.cells.findAt((self.Width*0.5, self.Length - 0.1, z)).index
            index5 = self.soilPart.cells.findAt((0.1, self.Length*0.5, z)).index
            section_name = name + "_Section"

            self.soilModel.HomogeneousSolidSection(name=section_name, material=name)
            self.soilPart.SectionAssignment(region=(self.soilPart.cells[index1], self.soilPart.cells[index2], self.soilPart.cells[index3],
                                                    self.soilPart.cells[index4], self.soilPart.cells[index5]), sectionName=section_name)
            self.soilPart.Set(cells=self.f(self.soilPart.cells, [index1, index2, index3, index4, index5]), name=name + "_Cell")

        name = self.parameter["Name"][-1]

        index1 = self.soilPart.cells.findAt((self.Width*0.5, self.Length*0.5, 0.1)).index
        index2 = self.soilPart.cells.findAt((self.Width*0.5, 0.1, 0.1)).index
        index3 = self.soilPart.cells.findAt((self.Width - 0.1, self.Length*0.5, 0.1)).index
        index4 = self.soilPart.cells.findAt((self.Width*0.5, self.Length - 0.1, 0.1)).index
        index5 = self.soilPart.cells.findAt((0.1, self.Length*0.5, 0.1)).index
        section_name = name + "_Section"


        self.soilModel.HomogeneousSolidSection(name=section_name, material=name)
        self.soilPart.SectionAssignment(region=(self.soilPart.cells[index1], self.soilPart.cells[index2], self.soilPart.cells[index3],
                                                    self.soilPart.cells[index4], self.soilPart.cells[index5]), sectionName=section_name)
    def create_edge_set(self):
        single_seed_edges = []
        infinite_cells = []
        coarse_edges = []
        heights = list(self.layer_heights) + [0]
        centers = self.centers + [self.inf_size_z/2]
        if len(self.SP_pattern)>0 and self.ditch_number==0:#SP
            self.soilPart.PartitionCellByExtrudeEdge(cells=self.soilPart.cells,edges=(
                self.soilPart.edges.findAt((self.Width*0.5 - self.ditch_length*0.5,self.y_border_Top-1,self.Depth)),
                self.soilPart.edges.findAt((self.Width*0.5 - self.ditch_length*0.5,self.SP_pattern[0]+self.SP_thickness*0.5,self.Depth)),
                self.soilPart.edges.findAt((self.Width*0.5 - self.ditch_length*0.5,self.free_space_Y+self.source_size+1,self.Depth)),
                self.soilPart.edges.findAt((self.Width*0.5 - self.ditch_length*0.5,self.free_space_Y+self.source_size*0.5,self.Depth)),
                self.soilPart.edges.findAt((self.Width*0.5 - self.ditch_length*0.5,self.y_border_Bottom+0.1,self.Depth))),
                line=self.soilPart.edges.findAt((0,self.inf_size_y,0.1)),sense=REVERSE)
            
            self.soilPart.PartitionCellByExtrudeEdge(cells=self.soilPart.cells,edges=(
                self.soilPart.edges.findAt((self.Width*0.5 + self.ditch_length*0.5,self.y_border_Top-1,self.Depth)),
                self.soilPart.edges.findAt((self.Width*0.5 + self.ditch_length*0.5,self.SP_pattern[0]+self.SP_thickness*0.5,self.Depth)),
                self.soilPart.edges.findAt((self.Width*0.5 + self.ditch_length*0.5,self.free_space_Y+self.source_size+1,self.Depth)),
                self.soilPart.edges.findAt((self.Width*0.5 + self.ditch_length*0.5,self.free_space_Y+self.source_size*0.5,self.Depth)),
                self.soilPart.edges.findAt((self.Width*0.5 + self.ditch_length*0.5,self.y_border_Bottom+0.1,self.Depth))),
                line=self.soilPart.edges.findAt((0,self.inf_size_y,0.1)),sense=REVERSE)
            heights.append(self.Depth - self.SP_height)
            centers.append(self.Depth - self.SP_height-0.01)
        elif self.ditch_number > 0 and len(self.SP_pattern) == 0: #OT & RC
            self.soilPart.PartitionCellByExtrudeEdge(cells=self.soilPart.cells,edges=(
                self.soilPart.edges.findAt((self.Width*0.5 - self.ditch_length*0.5,self.y_border_Top-1,self.Depth)),
                self.soilPart.edges.findAt((self.Width*0.5 - self.ditch_length*0.5,self.free_space_Y+self.source_size+self.ditch2source+self.ditch_width*0.5,self.Depth)),
                self.soilPart.edges.findAt((self.Width*0.5 - self.ditch_length*0.5,self.free_space_Y+self.source_size+1,self.Depth)),
                self.soilPart.edges.findAt((self.Width*0.5 - self.ditch_length*0.5,self.free_space_Y+self.source_size*0.5,self.Depth)),
                self.soilPart.edges.findAt((self.Width*0.5 - self.ditch_length*0.5,self.y_border_Bottom+0.1,self.Depth))),
                line=self.soilPart.edges.findAt((0,self.inf_size_y,0.1)),sense=REVERSE)

            self.soilPart.PartitionCellByExtrudeEdge(cells=self.soilPart.cells,edges=(
                self.soilPart.edges.findAt((self.Width*0.5 + self.ditch_length*0.5,self.y_border_Top-1,self.Depth)),
                self.soilPart.edges.findAt((self.Width*0.5 + self.ditch_length*0.5,self.free_space_Y+self.source_size+self.ditch2source+self.ditch_width*0.5,self.Depth)),
                self.soilPart.edges.findAt((self.Width*0.5 + self.ditch_length*0.5,self.free_space_Y+self.source_size+1,self.Depth)),
                self.soilPart.edges.findAt((self.Width*0.5 + self.ditch_length*0.5,self.free_space_Y+self.source_size*0.5,self.Depth)),
                self.soilPart.edges.findAt((self.Width*0.5 + self.ditch_length*0.5,self.y_border_Bottom+0.1,self.Depth))),
                line=self.soilPart.edges.findAt((0,self.inf_size_y,0.1)),sense=REVERSE)
            heights.append(self.Depth - self.ditch_height)
            centers.append(self.Depth - self.ditch_height-0.01)
        elif len(self.SP_pattern)>0 and self.ditch_number>0: #SP-OT & SP-RC
            self.soilPart.PartitionCellByExtrudeEdge(cells=self.soilPart.cells,edges=(
                self.soilPart.edges.findAt((self.Width*0.5 - self.ditch_length*0.5,self.y_border_Top-1,self.Depth)),
                self.soilPart.edges.findAt((self.Width*0.5 - self.ditch_length*0.5,self.SP_pattern[0]+self.SP_thickness*0.5,self.Depth)),
                self.soilPart.edges.findAt((self.Width*0.5 - self.ditch_length*0.5,self.free_space_Y+self.source_size+self.ditch2source+self.ditch_width*0.5,self.Depth)),
                self.soilPart.edges.findAt((self.Width*0.5 - self.ditch_length*0.5,self.free_space_Y+self.source_size+1,self.Depth)),
                self.soilPart.edges.findAt((self.Width*0.5 - self.ditch_length*0.5,self.free_space_Y+self.source_size*0.5,self.Depth)),
                self.soilPart.edges.findAt((self.Width*0.5 - self.ditch_length*0.5,self.y_border_Bottom+0.1,self.Depth))),
                line=self.soilPart.edges.findAt((0,self.inf_size_y,0.1)),sense=REVERSE)

            self.soilPart.PartitionCellByExtrudeEdge(cells=self.soilPart.cells,edges=(
                self.soilPart.edges.findAt((self.Width*0.5 + self.ditch_length*0.5,self.y_border_Top-1,self.Depth)),
                self.soilPart.edges.findAt((self.Width*0.5 + self.ditch_length*0.5,self.SP_pattern[0]+self.SP_thickness*0.5,self.Depth)),
                self.soilPart.edges.findAt((self.Width*0.5 + self.ditch_length*0.5,self.free_space_Y+self.source_size+self.ditch2source+self.ditch_width*0.5,self.Depth)),
                self.soilPart.edges.findAt((self.Width*0.5 + self.ditch_length*0.5,self.free_space_Y+self.source_size+1,self.Depth)),
                self.soilPart.edges.findAt((self.Width*0.5 + self.ditch_length*0.5,self.free_space_Y+self.source_size*0.5,self.Depth)),
                self.soilPart.edges.findAt((self.Width*0.5 + self.ditch_length*0.5,self.y_border_Bottom+0.1,self.Depth))),
                line=self.soilPart.edges.findAt((0,self.inf_size_y,0.1)),sense=REVERSE)
            heights.append(self.Depth - self.ditch_height)
            centers.append(self.Depth - self.ditch_height-0.01)
        


        if len(self.SP_pattern)>0:
            heights.append(self.Depth - self.SP_height)
            centers.append(self.Depth - self.SP_height+0.01)

        if self.ditch_number > 0:
            heights.append(self.Depth - self.ditch_height)
            centers.append(self.Depth - self.ditch_height+0.01)

        for h in centers:
            cells = [(float(self.Width*0.5),float(self.Length-0.1),float(h)),
                    (float(self.Width-1),float(self.Length*0.5),float(h)),
                    (float(self.Width*0.5),float(0.1),float(h)),
                    (0.1,float(self.Length*0.5),float(h))]

            for N in cells:
                infinite_cells.append(self.soilPart.cells.findAt(N).index)

        for z in heights:
            lines = [(self.x_border_Left*0.5,self.y_border_Bottom,z),
                    (self.x_border_Right+0.1,self.y_border_Bottom,z),
                    (self.x_border_Left*0.5,self.y_border_Top,z),
                    (self.x_border_Right+0.1,self.y_border_Top,z),
                    (self.x_border_Left,self.y_border_Top+0.1,z),
                    (self.x_border_Left,self.y_border_Bottom*0.5,z),
                    (self.x_border_Right,self.y_border_Top+0.1,z),
                    (self.x_border_Right,self.y_border_Bottom*0.5,z)]

            for node in lines:
                single_seed_edges.append(self.soilPart.edges.findAt(node).index)

        coarse_edges.append(self.soilPart.edges.findAt((self.x_border_Left,self.y_border_Bottom,0.01)).index)
        coarse_edges.append(self.soilPart.edges.findAt((self.x_border_Right,self.y_border_Bottom,0.01)).index)
        coarse_edges.append(self.soilPart.edges.findAt((self.x_border_Right,self.y_border_Top,0.01)).index)
        coarse_edges.append(self.soilPart.edges.findAt((self.x_border_Left,self.y_border_Top,0.01)).index)
        coarse_edges.append(self.soilPart.edges.findAt((0,self.y_border_Bottom,0.01)).index)
        coarse_edges.append(self.soilPart.edges.findAt((self.Width,self.y_border_Bottom,0.01)).index)
        coarse_edges.append(self.soilPart.edges.findAt((self.Width,self.y_border_Top,0.01)).index)
        coarse_edges.append(self.soilPart.edges.findAt((0,self.y_border_Top,0.01)).index)
        coarse_edges.append(self.soilPart.edges.findAt((self.x_border_Left,0,0.01)).index)
        coarse_edges.append(self.soilPart.edges.findAt((self.x_border_Left,self.Length,0.01)).index)
        coarse_edges.append(self.soilPart.edges.findAt((self.x_border_Right,self.Length,0.01)).index)
        coarse_edges.append(self.soilPart.edges.findAt((self.x_border_Right,0,0.01)).index)

        self.soilPart.Set(edges=self.f(self.soilPart.edges,single_seed_edges), name="Single_Seed_Edges")
        self.soilPart.Set(edges=self.f(self.soilPart.edges,coarse_edges), name="Coarse_Edges")
        self.soilPart.Set(cells=self.f(self.soilPart.cells, infinite_cells), name="Inf_Cells")
    def create_face_sets(self):
        NZ = (self.Width / 2, self.Length*0.5, 0)
        Z = self.soilPart.faces.findAt(NZ).index
        self.soilPart.Set(faces=self.f(self.soilPart.faces, [Z]), name="Z_Face")

    def create_datum_planes(self):
        edge = self.soilPart.edges.findAt((self.Width, self.Length / 2, self.Depth))
        self.soilModel.ConstrainedSketch(gridSpacing=2.37, name='__profile__',
                                        sheetSize=self.Width, transform=self.soilPart.MakeSketchTransform(
                                        sketchPlane=self.soilPart.faces.findAt((self.Width*0.5, self.Length*0.5, self.Depth)),
                                        sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(0, 0, self.Depth), sketchUpEdge=edge))

        self.soilPart.projectReferencesOntoSketch(filter=COPLANAR_EDGES, sketch=self.soilModel.sketches['__profile__'])

        self.soilModel.sketches['__profile__'].rectangle(point1=(0, self.free_space_Y), point2=(self.Width, self.free_space_Y + self.source_size))
        self.soilModel.sketches['__profile__'].Line(point1=(self.Width*0.5, 0), point2=(self.Width*0.5, self.Length))
        if self.ditch_number > 0 or len(self.SP_pattern)>0:
            self.soilModel.sketches['__profile__'].rectangle(point1=(float(self.Width*0.5 - self.ditch_length*0.5), 0),
                                                             point2=(float(self.Width*0.5 + self.ditch_length*0.5), self.Length))
        self.soilPart.PartitionFaceBySketch(faces=self.soilPart.faces.findAt((self.Width / 2, self.Length / 2, self.Depth)),
            sketch=self.soilModel.sketches['__profile__'], sketchUpEdge=edge)

    def draw_source(self):
        N1 = (self.free_space_X, self.free_space_Y, self.Depth)
        N2 = (self.free_space_X + self.source_size,self.free_space_Y, self.Depth)
        N3 = (self.free_space_X + self.source_size,self.free_space_Y + self.source_size, self.Depth)
        N4 = (self.free_space_X, self.free_space_Y +self.source_size, self.Depth)

        for i in [N1, N2, N3, N4]:
            self.soilPart.DatumPointByCoordinate(i)

        face = self.soilPart.faces.findAt(N1)
        self.soilPart.PartitionFaceByShortestPath(face, N1, N2)
        self.soilPart.PartitionFaceByShortestPath(face, N2, N3)
        self.soilPart.PartitionFaceByShortestPath(face, N3, N4)
        self.soilPart.PartitionFaceByShortestPath(face, N4, N1)
        Y = self.soilPart.faces.findAt((self.free_space_X+0.01, self.free_space_Y+0.01, self.Depth)).index
        self.soilPart.Set(faces=self.f(self.soilPart.faces, [Y]), name="Source Face")
        self.soilPart.Surface(name='Source Surface', side1Faces= self.f(self.soilPart.faces, [Y]))

    def draw_ditches(self, ditch_number):
        x1 = (self.Width - self.ditch_length)*0.5
        x2 = x1 + self.ditch_length

        y1 = self.free_space_Y + self.source_size + self.ditch2source + (ditch_number-1)*(self.ditch_width+self.ditch2ditch)

        y2 = y1 + self.ditch_width
        top_face = self.soilPart.faces.findAt((self.Width*0.5, self.Length*0.5, self.Depth))
        edge_index = self.soilPart.edges.findAt((self.Width, self.Length*0.5, self.Depth)).index
        self.soilModel.ConstrainedSketch(name="__profile__", sheetSize=self.Width * 2,
                                        transform=self.soilPart.MakeSketchTransform(sketchPlane=top_face, origin=(0, 0, self.Depth),
                                        sketchPlaneSide=SIDE1, sketchUpEdge=self.soilPart.edges[edge_index], sketchOrientation=RIGHT))
        sketch = self.soilModel.sketches['__profile__']
        self.soilPart.projectReferencesOntoSketch(filter=COPLANAR_EDGES, sketch=sketch)
        sketch.rectangle(point1=(x1, float(y1)), point2=(x2, float(y2)))
        self.soilPart.CutExtrude(depth=self.ditch_height, flipExtrudeDirection=OFF, sketch=sketch,
                                 sketchOrientation=RIGHT, sketchPlane=top_face, sketchPlaneSide=SIDE1,
                                 sketchUpEdge=self.soilPart.edges[edge_index])
        del self.soilModel.sketches['__profile__']

    def create_SP(self):
        # Extrude SP Cell
        if len(self.SP_pattern)>0:
            for y1 in self.SP_pattern:
                x1 = (self.Width - self.ditch_length)*0.5
                x2 = x1 + self.ditch_length

                y2 = y1 + self.SP_thickness
                top_face = self.soilPart.faces.findAt((self.Width*0.5, self.Length*0.5, self.Depth))
                edge_index = self.soilPart.edges.findAt((self.Width, self.Length*0.5, self.Depth)).index
                self.soilModel.ConstrainedSketch(name="__profile__", sheetSize=self.Width * 2,
                                                transform=self.soilPart.MakeSketchTransform(sketchPlane=top_face, origin=(0, 0, self.Depth),
                                                sketchPlaneSide=SIDE1, sketchUpEdge=self.soilPart.edges[edge_index], sketchOrientation=RIGHT))

                sketch = self.soilModel.sketches['__profile__']
                self.soilPart.projectReferencesOntoSketch(filter=COPLANAR_EDGES, sketch=sketch)
                sketch.rectangle(point1=(x1, float(y1)), point2=(x2, float(y2)))
                self.soilPart.CutExtrude(depth=self.SP_height, flipExtrudeDirection=OFF, sketch=sketch,sketchOrientation=RIGHT,
                    sketchPlane=top_face, sketchPlaneSide=SIDE1,sketchUpEdge=self.soilPart.edges[edge_index])

                del self.soilModel.sketches['__profile__']

                self.soilPart.PartitionCellByExtrudeEdge(cells=self.soilPart.cells,edges=(
                    self.soilPart.edges.findAt((self.Width*0.5 - self.ditch_length*0.5,y1+self.SP_thickness*0.5,self.Depth))),
                    line=self.soilPart.edges.findAt((0,self.inf_size_y,0.1)),sense=REVERSE)
                
                self.soilPart.PartitionCellByExtrudeEdge(cells=self.soilPart.cells,edges=(
                    self.soilPart.edges.findAt((self.Width*0.5 + self.ditch_length*0.5,y1+self.SP_thickness*0.5,self.Depth))),
                    line=self.soilPart.edges.findAt((0,self.inf_size_y,0.1)),sense=REVERSE)

            self.partition_cell(self.Depth - self.SP_height)
            #Create Part
            soilProfileSketch = self.soilModel.ConstrainedSketch(name="__profile__", sheetSize=self.Width * 2)

            soilProfileSketch.rectangle(point1=(0,0),point2=(self.ditch_length,self.SP_thickness))

            self.SPPart = self.soilModel.Part(name="Sheet Pile", dimensionality=THREE_D, type=DEFORMABLE_BODY)
            self.SPPart.BaseSolidExtrude(sketch=soilProfileSketch, depth=self.SP_height)

            del self.soilModel.sketches['__profile__']

            #Create Material
            name = "Steel"
            density = self.SP_density
            elasticity = (self.SP_E, 0.3)
            damping_ratio = 0

            self.create_material(name, {"E": elasticity, "D": density})

            #Create Section
            self.soilModel.HomogeneousSolidSection(name="SP_Section", material="Steel")
            self.SPPart.SectionAssignment(region=(self.SPPart.cells[0],), sectionName="SP_Section")

            #Create instances
            self.soilModel.rootAssembly.DatumCsysByDefault(CARTESIAN)
            self.soilModel.rootAssembly.Instance(dependent=ON, name="Sheet Pile-1", part=self.SPPart)
            self.soilModel.rootAssembly.regenerate()
            self.soilModel.rootAssembly.translate(instanceList=('Sheet Pile-1', ),
                vector=((self.Width-self.ditch_length)*0.5, self.SP_pattern[0], self.Depth-self.SP_height))

            #Create Interaction
            """try:
                self.soilModel.ContactProperty('IntProp-1')
                self.soilModel.interactionProperties['IntProp-1'].NormalBehavior(allowSeparation=ON, constraintEnforcementMethod=DEFAULT,pressureOverclosure=HARD)
                self.soilModel.ContactStd(createStepName='Initial', name='Int-1')
                self.soilModel.interactions['Int-1'].includedPairs.setValuesInStep(stepName='Initial', useAllstar=ON)
                self.soilModel.interactions['Int-1'].contactPropertyAssignments.appendInStep(assignments=((GLOBAL, SELF, 'IntProp-1'), ), stepName='Initial')
            except:
                pass"""
            #Create Mesh
            self.SPPart.seedPart(size=self.mesh_size)
            self.SPPart.generateMesh()

    def fillDitch(self):
        #Create Part
        ProfileSketch = self.soilModel.ConstrainedSketch(name="__profile__", sheetSize=self.Width * 2)
        ProfileSketch.rectangle(point1=(0,0),point2=(self.ditch_length,self.ditch_width))
        self.RCPart = self.soilModel.Part(name="Rubber Chip", dimensionality=THREE_D, type=DEFORMABLE_BODY)
        self.RCPart.BaseSolidExtrude(sketch=ProfileSketch, depth=self.ditch_height)
        del self.soilModel.sketches['__profile__']

        #Create Material
        name = "Rubber"
        density = self.RC_density
        elasticity = (self.RC_E, 0.25)

        self.create_material(name, {"E": elasticity, "D": density,"Damping": self.RC_damping, "Vs": self.RC_VS, "H": self.ditch_height})

        #Create Section
        self.soilModel.HomogeneousSolidSection(name="RC_Section", material="Rubber")
        self.RCPart.SectionAssignment(region=(self.RCPart.cells[0],), sectionName="RC_Section")

        #Create instances
        self.soilModel.rootAssembly.DatumCsysByDefault(CARTESIAN)
        self.soilModel.rootAssembly.Instance(dependent=ON, name="Rubber Chips-1", part=self.RCPart)
        self.soilModel.rootAssembly.regenerate()
        self.soilModel.rootAssembly.translate(instanceList=('Rubber Chips-1', ),
            vector=((self.Width-self.ditch_length)*0.5, self.free_space_Y+self.source_size+self.ditch2source, self.Depth-self.ditch_height))

        #Create Interaction
        try:
            self.soilModel.ContactProperty('IntProp-1')
            self.soilModel.interactionProperties['IntProp-1'].NormalBehavior(allowSeparation=ON, constraintEnforcementMethod=DEFAULT,pressureOverclosure=HARD)
            self.soilModel.ContactStd(createStepName='Initial', name='Int-1')
            self.soilModel.interactions['Int-1'].includedPairs.setValuesInStep(stepName='Initial', useAllstar=ON)
            self.soilModel.interactions['Int-1'].contactPropertyAssignments.appendInStep(assignments=((GLOBAL, SELF, 'IntProp-1'), ), stepName='Initial')
        except:
            pass

        #Create Mesh
        self.RCPart.seedPart(size=self.mesh_size)
        self.RCPart.generateMesh()

    def create_ditch(self):
        if self.ditch_number > 0:
            if self.Depth - self.ditch_height != self.parameter["Thicknesses"][0]:
                d_bottom = self.soilPart.DatumPlaneByPrincipalPlane(XYPLANE, self.Depth - self.ditch_height).id
                self.soilPart.PartitionCellByDatumPlane(self.soilPart.cells, self.soilPart.datums[d_bottom])

            """for i in range(1, self.ditch_number + 1):
                Y1 = self.free_space_Y + self.source_size + self.ditch2source + (i - 1) * (self.ditch_width + self.ditch2ditch)
                Y2 = Y1 + self.ditch_width
                for i in [Y1, Y2]:
                    cells = self.soilPart.cells.getByBoundingBox(0, 0, self.Depth - self.ditch_height, self.Width,
                                                                 self.Length, self.Depth)
                    dz = self.soilPart.DatumPlaneByPrincipalPlane(XZPLANE, i).id
                    self.soilPart.PartitionCellByDatumPlane(cells, self.soilPart.datums[dz])

            for i in [x1, x2]:
                cells = self.soilPart.cells.getByBoundingBox(0, 0, self.Depth - self.ditch_height, self.Width, self.Length,
                                                             self.Depth)
                dx = self.soilPart.DatumPlaneByPrincipalPlane(YZPLANE, i).id
                self.soilPart.PartitionCellByDatumPlane(cells, self.soilPart.datums[dx])"""

            for i in range(1, self.ditch_number + 1):
                self.draw_ditches(i)

        if self.fill_ditch:
            self.fillDitch()

    def create_instance(self):
        self.soilModel.rootAssembly.DatumCsysByDefault(CARTESIAN)
        self.soilModel.rootAssembly.Instance(dependent=ON, name="Soil Part-1", part=self.soilPart)

    def meshControl(self):
        heights = list(self.layer_heights)
        centers = sorted(self.centers,reverse=True)
        if self.ditch_number > 0:
            heights.append(self.Depth - self.ditch_height)

        if len(self.SP_pattern)>0:
            heights.append(self.Depth - self.SP_height)

        sh = sorted(heights,reverse=True)
        
        for n in range(len(heights)):
            z = sh[n] - 0.01
            print(z)
            h = sh[n]
            if n == 0:
                cells_edges = [
                    [(self.Width*0.5,self.Length-0.1,z),(self.x_border_Right,self.Length-0.1,h),REVERSE],
                    [(self.Width*0.5,0.1,z),(self.x_border_Right,0.1,h),FORWARD],
                    [(self.Width-0.1,self.Length*0.5,z),(self.x_border_Right+0.1,self.y_border_Bottom,h),REVERSE],
                    [(0.1,self.Length*0.5,z),(0.1,self.y_border_Bottom,h),FORWARD]]
            else:
                cells_edges = [
                    [(self.Width*0.5,self.Length-0.1,z),(self.x_border_Right,self.Length-0.1,h),FORWARD],
                    [(self.Width*0.5,0.1,z),(self.x_border_Right,0.1,h),REVERSE],
                    [(self.Width-0.1,self.Length*0.5,z),(self.x_border_Right+0.1,self.y_border_Bottom,h),FORWARD],
                    [(0.1,self.Length*0.5,z),(0.1,self.y_border_Bottom,h),REVERSE]]

            for region in cells_edges:
                cell = self.soilPart.cells.findAt(region[0])
                edge = self.soilPart.edges.findAt(region[1])
                self.soilPart.setSweepPath(edge=edge, region=cell, sense=region[2])

    def create_mesh(self):
        self.soilPart.setMeshControls(elemShape=HEX, regions=self.soilPart.cells, technique=STRUCTURED)
        self.soilPart.setMeshControls(elemShape=HEX, regions=self.soilPart.sets["Inf_Cells"].cells, technique=SWEEP)
        self.soilPart.setElementType(elemTypes=(mesh.ElemType(elemCode=C3D8R, elemLibrary=STANDARD),),regions=(self.soilPart.cells,))
        self.soilPart.setElementType(elemTypes=(mesh.ElemType(elemCode=AC3D8, elemLibrary=STANDARD),),regions=(self.soilPart.sets["Inf_Cells"].cells,))
        self.meshControl()
        # Vertical Meshing
        #self.soilPart.seedEdgeBySize(edges=self.soilPart.sets["Coarse_Edges"].edges, size=1,constraint=FIXED)
        self.soilPart.seedPart(size=self.mesh_size)
        self.soilPart.seedEdgeByNumber(edges=self.soilPart.sets["Single_Seed_Edges"].edges, number=1,constraint=FIXED)
        self.soilPart.seedEdgeByBias(biasMethod=SINGLE, constraint=FIXED, end2Edges=self.soilPart.sets["Coarse_Edges"].edges, maxSize=1.0, minSize=self.mesh_size)
        # Surface Meshing
        # Y-axis
        #self.soilPart.seedEdgeBySize(edges=self.soilPart.sets["SurfaceEdges"].edges,size = self.mesh_size)
        #self.soilPart.seedEdgeBySize(edges=self.soilPart.sets["HorizontalEdges"].edges,size = self.mesh_size)
        #for edge in self.soilPart.sets["HorizontalEdges"].edges:
        #    self.soilPart.setSeedConstraints(constraint=FIXED, edges=(edge,))
        self.soilPart.generateMesh()

        # self.soilPart.seedEdgeByBias(biasMethod=DOUBLE, endEdges=self.soilPart.sets["TopEdge"].edges, minSize=float(self.mesh_size), maxSize=float(self.max_mesh_size))

    def create_nodes(self):
        self.node_list = []
        x2 = self.Width / 2
        y2 = self.free_space_Y + self.source_size + array(self.accelometer_pattern)
        for i in y2:
            self.node_list.append(self.soilPart.nodes.getClosest((x2, i, self.Depth)))
            self.node_list.append(self.soilPart.nodes.getClosest((x2, i, self.Depth)))

        self.soilPart.Set(nodes=mesh.MeshNodeArray(self.node_list), name="Accelometers")

    def create_vibration(self):
        time = arange(0, self.duration + self.time_step, self.time_step)
        accelerations = self.PGA * 9.81 * np.sin(2 * pi * self.frequency * time)

        self.data = [[time[i], accelerations[i]] for i in range(len(time))]

    def create_step(self):
        self.create_vibration()
        #self.soilModel.ImplicitDynamicsStep(initialInc=self.time_step, timePeriod=self.duration, maxInc=self.time_step,
        #                                    maxNumInc=int(2 * (self.duration / self.time_step)), name='Vibration Step',
        #                                    previous='Initial')
        self.soilModel.ExplicitDynamicsStep(improvedDtMethod=ON, name='Vibration Step', previous='Initial', 
            timeIncrementationMethod=FIXED_USER_DEFINED_INC, userDefinedInc=0.0005)
        self.soilModel.TabularAmplitude(name="Vibration", timeSpan=STEP, smooth=SOLVER_DEFAULT, data=self.data)

    def create_boundary_conditions(self):
        Sets = self.soilModel.rootAssembly.instances["Soil Part-1"].sets
        self.soilModel.DisplacementBC(createStepName='Initial', name='BC-Z',
                                      region=self.soilModel.rootAssembly.instances["Soil Part-1"].sets['Z_Face'], u1=0,u2=0, u3=0)
        #self.soilModel.AccelerationBC(amplitude="Vibration", createStepName='Vibration Step', name='Vibration', region=Sets['Source Face'], a3=-1)
        self.soilModel.Pressure(amplitude='Vibration', createStepName='Vibration Step', magnitude=1,name='Vibration',
                                region=self.soilModel.rootAssembly.instances['Soil Part-1'].surfaces['Source Surface'])

    def create_history_output(self):
        self.soilModel.fieldOutputRequests["F-Output-1"].deactivate("Vibration Step")
        self.soilModel.HistoryOutputRequest(createStepName="Vibration Step", frequency=1, name="H-Output-2",variables=('A3','V3'),
                                            region=self.soilModel.rootAssembly.allInstances['Soil Part-1'].sets['Accelometers'])

        del self.soilModel.historyOutputRequests['H-Output-1']
        del self.soilModel.fieldOutputRequests['F-Output-1']

    def create_job(self):
        self.job_name = self.model_name
        mdb.Job(model=self.model_name, name=self.job_name, type=ANALYSIS, memory=90, memoryUnits=PERCENTAGE, numCpus=12,numDomains=12, numGPUs=1)
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
        self.layer_centers()

        for i in range(len(self.parameter["Name"])):
            name = self.parameter["Name"][i]
            height = sum(self.parameter["Thicknesses"][:i + 1])
            density = self.parameter["Density"][i]
            thickness = self.parameter["Thicknesses"][i]
            Vs = self.parameter["Vs"][i]
            elasticity = (self.parameter["Elastic"][i], self.parameter["Poisson"][i])
            damping_ratio = self.parameter["Damping Ratio"]

            self.partition_cell(self.Depth - height)
            self.create_material(name, {"E": elasticity, "D": density, "Damping": damping_ratio, "Vs": Vs, "H": thickness})

        self.create_section()
        self.draw_source()
        self.create_ditch()
        self.create_SP()
        self.create_datum_planes()

        self.create_instance()
        self.create_edge_set()

        self.create_mesh()

        self.create_nodes()
        self.create_step()
        self.create_boundary_conditions()
        self.create_history_output()
        self.create_job()
        self.change_element_type()


model = Create_Model()
model.operator()
