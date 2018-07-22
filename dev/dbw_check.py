import pandas as pd
import os
import matplotlib.pyplot as plt

class DBW_Checker:
	def __init__(self, DATA_DIR = "../ros/src/twist_controller/", FILES = ["brakes.csv", "steers.csv", "throttles.csv"]):
		self.DATA_DIR = DATA_DIR
		self.FILES = FILES
		
	def check(self, file = None):
		if file is None:
			file = self.FILES
		else:
			file = [file]
		
		for filename in file:
			self._process_file(os.path.join(self.DATA_DIR, filename), filename)
		
	def _process_file(self, filename, title):
		df = pd.read_csv(filename)
		df["err"] = df["actual"] - df["proposed"]
		print(df.describe())
		plt.figure()
		df["err"].plot(title = title)
		print(title, ((df.actual - df.proposed)**2).mean())