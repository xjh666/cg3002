import pandas as pd
import numpy as np
from feature_extract import extract_features
from os import listdir

window_size = 50
step_size = 10
dance_data = np.array([[0.0]*91])
for f in listdir("raw_data"):
	if ".txt" in f:
		dataframe = pd.read_csv("raw_data/"+f, sep=",", header=None)
		dataset = dataframe.values
		print(f)
		i = 0
		while i+window_size <= len(dataset): 
			X = dataset[i:i+window_size,0:15]
			X = extract_features(X)
			if "chicken" in f:
				X = np.append(X, 0)
			if "cowboy_front" in f:
				X = np.append(X, 1)
			if "cowboy_back" in f:
				X = np.append(X, 2)
			if "crab" in f:
				X = np.append(X, 3)
			if "hunchback" in f:
				X = np.append(X, 4)
			if "raffles_left" in f:
				X = np.append(X, 5)
			if "raffles_right" in f:
				X = np.append(X, 6)
			if "running" in f:
				X = np.append(X, 7)
			if "james" in f:
				X = np.append(X, 8)
			if "snake" in f:
				X = np.append(X, 9)
			if "double" in f:
				X = np.append(X, 10)
			if "mermaid" in f:
				X = np.append(X, 11)
			if "final" in f:
				X = np.append(X, 12)
			dance_data = np.append(dance_data, [X], axis=0)
			i += step_size
dance_data = dance_data[1:]
np.savetxt("final.csv", dance_data, delimiter=",")
