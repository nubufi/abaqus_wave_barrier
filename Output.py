from abaqus import *
from abaqusConstants import *
from caeModules import *
import regionToolset
from odbAccess import *
from os.path import isfile, join
from time import sleep
import os


class Output:
    def __init__(self, model_name, target_path, dimension):
        self.model_name = model_name
        self.job_name = model_name
        self.target_path = target_path
        self.dimension = dimension

    def check_file(self):
        file_name = self.job_name + ".lck"
        while isfile(file_name) == True:
            sleep(0.1)
        file_name = self.job_name + ".odb"
        self.odb = openOdb(path=join(self.target_path, file_name))

    def row2col(self, data):
        temp_time = []
        temp_acc = []

        for d in data:
            temp_time.append(round(d[0], 4))
            temp_acc.append(round(d[1], 10))

        return temp_time, temp_acc

    def read_history_odb(self):
        acc_type = "A2" if self.dimension == "2D" else "A3"
        vel_type = "V2" if self.dimension == "2D" else "V3"
        sleep(5)
        self.check_file()
        step = self.odb.steps["Vibration Step"]
        acc_output = {}
        vel_output = {}
        accelometers = step.historyRegions.keys()
        accelometers.sort()
        for acc in accelometers:
            accData = step.historyRegions[acc].historyOutputs[acc_type].data
            velData = step.historyRegions[acc].historyOutputs[vel_type].data
            time, accelerations = self.row2col(accData)
            time, velocities = self.row2col(velData)
            acc_output["Time"] = time
            acc_output[acc] = accelerations
            vel_output["Time"] = time
            vel_output[acc] = velocities
        return acc_output, vel_output

    def wright_to_txt(self, data, outputType):
        f = open("{}_{}.txt".format(self.model_name, outputType), "w")
        keys = data.keys()
        del keys[keys.index("Time")]
        keys.sort()

        first_row = [i for i in keys]
        first_row.insert(0, "Time(s)             ")
        f.write("   ".join(first_row) + "\n")

        time = data["Time"]
        for i in range(len(time)):
            row_data = []
            row_data.append(str(time[i]) + " " * (25 - len(str(time[i]))))
            for key in keys:
                d = str(data[key][i])
                row_data.append(d + " " * (25 - len(d)))
            f.write("".join(row_data) + "\n")
        f.close()

    def operator(self):
        acc_data, vel_data = self.read_history_odb()
        self.wright_to_txt(acc_data, "Acc")
        self.wright_to_txt(vel_data, "Vel")


dimension = "temp_dimension"
model_name = "temp_model_name"
target_path = os.getcwd()
output_type = "temp_output"
output = Output(model_name, target_path, dimension)
output.operator()
