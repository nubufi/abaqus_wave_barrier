from os.path import isfile
from time import sleep
import os
from threading import Thread
from datetime import datetime
from shutil import move
import sys
from time import time
from shutil import copy2, move
from pandas import read_excel
from helper import excel_to_json, create_dir

TEMP_PATH = "C:\\Temp"


class Monitor:
    def __init__(self):
        self.initial_path = os.getcwd()

        model_parameters = (
            read_excel("input.xlsx", sheet_name="Model", header=None).iloc[:, 1].values
        )
        motion_parameters = (
            read_excel("input.xlsx", sheet_name="Motion", header=None).iloc[:, 1].values
        )

        self.model_name = model_parameters[0]
        self.dimension = model_parameters[4]

        self.time_step = motion_parameters[3]
        self.duration = motion_parameters[4]

        self.target_path = os.path.join(self.initial_path, self.model_name)
        self.file_name = self.model_name + ".sta"

        excel_to_json("input.xlsx", self.model_name)

        self.starting_time = time()

    def path_creator(self):
        create_dir(self.model_name)
        os.chdir(self.target_path)
        for d in ["ODB", "Output", os.path.join("Output", self.model_name)]:
            try:
                create_dir(d)
            except:
                continue
        os.chdir(self.initial_path)
        if self.dimension == "3D":
            copy2("D3.py", self.target_path)
        elif self.dimension == "2D":
            copy2("D2.py", self.target_path)
        else:
            copy2("D2_planar.py", self.target_path)

        copy2("model.py", self.model_name)
        copy2("data.py", self.model_name)

    def modify_output(self):
        old_path = os.path.join(self.initial_path, "Output.py")
        new_path = os.path.join(self.target_path, "Output.py")

        file_old = open(old_path)
        data = file_old.read().replace("temp_model_name", str(self.model_name))
        data = data.replace("temp_dimension", self.dimension)
        if self.dimension == "3D":
            data = data.replace("temp_output", "A3")
        else:
            data = data.replace("temp_output", "A2")

        file_old.close()

        new_file = open(new_path, "w")
        new_file.write(data)
        new_file.close()

    def check(self):
        while isfile(self.file_name) == False:
            sleep(0.1)

    def progress_bar(self, count, total):
        self.check()
        percents = int(100.0 * count / float(total))
        t = datetime.now().strftime("%H:%M:%S")
        hashes = "#" * percents
        spaces = " " * (100 - percents)
        sys.stdout.write(
            "\r {time} [{h}] {p}% ".format(time=t, h=(hashes + spaces), p=percents)
        )
        sys.stdout.flush()

    def read(self):
        self.check()
        count = 0
        c1 = 0
        total = self.duration / self.time_step
        while count <= total:
            f = open(self.file_name)
            lines = f.readlines()
            count = len(lines) - 5
            if c1 != count:
                self.progress_bar(count, total)
            c1 = count
            f.close()
            if lines[-1] == " THE ANALYSIS HAS NOT BEEN COMPLETED\n":
                sleep(5)
                self.is_killed = True
                sys.exit()

    def start_job(self, model_name):
        os.chdir(self.target_path)
        os.system("abaqus job={m} input={m} ask_delete=OFF".format(m=model_name))

    def folder(self):
        for i in os.listdir():
            ex = i.split(".")[-1]
            if ex == "odb":
                if i in os.listdir("ODB"):
                    os.remove("ODB\\{}".format(i))
                move(i, "ODB")
            elif ex == "txt":
                if i in os.listdir("Output\\{}".format(self.model_name)):
                    os.remove("Output\\{}\\{}".format(self.model_name, i))
                move(i, os.path.join("Output", self.model_name))
            else:
                try:
                    os.remove(i)
                except:
                    pass

    def output(self):
        if not self.is_killed:
            os.system("abaqus cae noGui={}".format("Output.py"))
        self.folder()
        os.chdir(os.path.join(self.target_path, "Output", self.model_name))
        os.chdir(self.initial_path)

    def operator(self):
        self.is_killed = False
        self.path_creator()
        self.modify_output()

        start_time = datetime.now().strftime("%H:%M:%S")
        print("Başlangıç Saati : ", start_time)
        os.chdir(self.target_path)
        if self.dimension == "3D":
            os.system("abaqus cae noGui={}".format("D3.py"))
        elif self.dimension == "2D":
            os.system("abaqus cae noGui={}".format("D2.py"))
        else:
            os.system("abaqus cae noGui={}".format("D2_planar.py"))

        print("Inp Oluşturulma Saati : ", str(datetime.now().strftime("%H:%M:%S")))
        self.start_job(self.model_name)
        reading = Thread(target=self.read)
        reading.daemon = True
        job = Thread(target=self.start_job, args=(self.model_name,))
        job.daemon = True

        reading.start()
        job.start()
        reading.join()
        job.join()
        self.output()


model = Monitor()
model.operator()
