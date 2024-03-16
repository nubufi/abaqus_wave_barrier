from os.path import isfile
from time import sleep
import os
from threading import Thread
from datetime import datetime
from shutil import move
import sys
from numpy import float,arange
from time import time
from model import Input
import json
from shutil import copy2,move

class Monitor:
    def __init__(self,input_obj:Input):
        self.initial_path = os.getcwd()
        
        self.input_obj = input_obj
        self.model_name = input_obj.model_name
        self.dimension = input_obj.dimension

        self.file_name = self.model_name + ".sta"
        self.startingTime = time()

    def path_creator(self):
        os.mkdir(self.model_name)
        self.target_path = os.getcwd(self.model_name)
        os.chdir(self.target_path)
        for d in ["ODB","Output",os.path.join("Output",self.model_name)]:
            try:
                os.mkdir(d)
            except:
                continue
        os.chdir(self.initial_path)
        if self.dimension == "3D":
            copy2("D3.py",self.target_path)
        elif self.dimension == "2D":
            copy2("D2.py",self.target_path)
        else:
            copy2("D2_planar.py",self.target_path)
        
        self.export_to_json()
        move("input.json",self.target_path)
        copy2("model.py",self.target_path)
    
    def export_to_json(self):
        with open(os.path.join(self.target_path,"input.json"),"w") as f:
            json.dump(self.input_obj.__dict__,f,indent=4)

    def modify_output(self):
        old_path = os.path.join(self.initial_path,"Output.py")
        new_path = os.path.join(self.target_path,"Output.py")

        file_old = open(old_path)
        data = file_old.read().replace("temp_model_name",str(self.model_name))
        data = data.replace("temp_dimension",self.Dimension)
        if self.Dimension == "3D":
            data = data.replace("temp_output","A3")
        else:
            data = data.replace("temp_output","A2")

        file_old.close()

        new_file = open(new_path,"w")
        new_file.write(data)
        new_file.close()

    def check(self):
        while isfile(self.file_name)==False:
            sleep(0.1)

    def progress_bar(self,count,total):
        self.check()
        percents = int(100.0 * count / float(total))
        t = datetime.now().strftime("%H:%M:%S")
        hashes = '#' * percents
        spaces = ' ' * (100 - percents)
        sys.stdout.write("\r {time} [{h}] {p}% ".format(time = t,h = (hashes + spaces), p = percents))
        sys.stdout.flush()

    def read(self):
        self.check()
        count = 0
        c1 = 0
        total = self.duration / self.time_step
        while count<=total:
            f = open(self.file_name)
            lines = f.readlines()
            count = len(lines) - 5
            if c1 != count:
                self.progress_bar(count, total)
            c1 = count
            f.close()
            duration = time() - self.startingTime
            if lines[-1]==" THE ANALYSIS HAS NOT BEEN COMPLETED\n":
                sleep(5)
                self.is_killed = True
                sys.exit()

    def start_job(self,model_name):
        os.chdir(self.target_path)
        os.system("abaqus job={m} input={m} ask_delete=OFF".format(m=model_name))

    def folder(self):
        for i in os.listdir():
            ex = i.split(".")[-1]
            if ex=="odb":
                if i in os.listdir("ODB"):
                    os.remove("ODB\\{}".format(i))
                move(i,"ODB")
            elif ex=="txt":
                if i in os.listdir("Output\\{}".format(self.model_name)):
                    os.remove("Output\\{}\\{}".format(self.model_name,i))
                move(i,os.path.join("Output",self.model_name))
            else:
                try:
                    os.remove(i)
                except:
                    pass

    def output(self):
        if not self.is_killed:
            os.system("abaqus cae noGui={}".format("Output.py"))
        self.folder()
        os.chdir(os.path.join(self.target_path,"Output",self.model_name))
        os.chdir(self.initial_path)

    def operator(self):
        self.is_killed = False
        self.path_creater()
        self.export_to_json()
        self.modify_output()

        start_time = datetime.now().strftime("%H:%M:%S")
        print("Başlangıç Saati : ",start_time)
        os.chdir(self.target_path)
        if self.dimension == "3D":
            os.system("abaqus cae noGui={}".format("D3.py"))
        elif self.dimension == "2D":
            os.system("abaqus cae noGui={}".format("D2.py"))
        else:
            os.system("abaqus cae noGui={}".format("D2_planar.py"))

        print("Inp Oluşturulma Saati : ", str(datetime.now().strftime("%H:%M:%S")))
        self.start_job(self.model_name)
        reading=Thread(target=self.read)
        reading.daemon=True
        job = Thread(target=self.start_job, args=(self.model_name,))
        job.daemon = True

        reading.start()
        job.start()
        reading.join()
        job.join()
        self.output()

pc_number = 0

dimension = "3D"
locations = ["Mentese","Milas","Senaryo1","Senaryo2","Senaryo3"]
layouts = ["L1","L2","L3","L4","L5"]
frequencies = arange(10,160,10)
exps = ["A","OT","RC","SP","SP-OT","SP-RC","RC-SP","OT-SP"]
patterns = {
    "L1":[1,1.5,2,3.5,6,8.5,11,13.5,16,18.5,21,25,29],
    "L2":[1,2,3.5,6,8.5,11,13.5,16,18.5,21,25,29],
    "L3":[1,2.5,6,8.5,11,13.5,16,18.5,21,25,29],
    "L4":[1,4.75,8.75,11,13.5,16,18.5,21,25,29],
    "L5":[1,6,8.5,11,13.5,16,18.5,25,29],
}

D2S = {
    "L1":2.25,
    "L2":4.75,
    "L3":7.25,
    "L4":9.75,
    "L5":12.25,
}



def expFunc(layout,exp):
    exps = {
        "A" : [0,0,0],
        "OT" : [1,0,0],
        "RC" : [1,1,0],
        "SP" : [0,0,D2S[layout]+0.75],
        "SP-OT" : [1,0,D2S[layout]+0.75],
        "SP-RC" : [1,1,D2S[layout]+0.75],
        "RC-SP" : [1,1,D2S[layout]],
        "OT-SP" : [1,0,D2S[layout]],
        "DT" : [2,0,0],
    }

    return exps[exp]

from itertools import product

combinations = list(product(locations,layouts,frequencies,exps))
def checkPcNumber():
    _,folder_name = os.path.split(os.path.dirname(os.getcwd()))
    return int(folder_name[4:])-1

def checkModel(model_name):
    if model_name not in os.listdir("..\Output"):
        return True
    elif len(os.listdir(f"..\Output\{model_name}")) == 0:
        return True
    else:
        return False

try:
    os.makedirs(os.path.join(os.path.dirname(os.getcwd()),"Output"))
except:
    pass

model = Monitor(dimension,"X",100,0,D2S["L2"],0,0,patterns["L1"],"Milas")
model.path_creater()
model.modify_model()
"""pc_number = checkPcNumber()
num_of_pc = 30
num_of_analysis = int(len(combinations)/num_of_pc)
start = num_of_analysis*pc_number
finish = min(start + num_of_analysis,len(combinations)) if pc_number != num_of_pc-1 else len(combinations)
for i in range(start,finish):
    location,layout,frequency,exp = combinations[i]
    model_name = f"{location}_{exp}_{layout}_{frequency}Hz_{dimension}"
    if checkModel(model_name):
        print(model_name)
        DN,RC,SP = expFunc(layout,exp)
        if exp == "RC-SP" or exp=="RC-OT":
            d2s = D2S[layout] + 0.05
        else:
            d2s = D2S[layout]
        model = Monitor(dimension,model_name,frequency,DN,d2s,SP,RC,patterns[layout],location)
        model.operator()"""
