import numpy as np
from scipy import stats

def compute_min(input_data):
	min = []
	for i in range(15):
		min.append(np.min(input_data[:,i]))
	return min

def compute_max(input_data):
	max = []
	for i in range(15):
		max.append(np.max(input_data[:,i]))
	return max

def compute_mean(input_data):
	mean = []
	for i in range(15):
		mean.append(np.mean(input_data[:,i]))
	return mean

def compute_standard_dev(input_data):
	standard_dev = []
	for i in range(15):
		standard_dev.append(np.std(input_data[:,i]))
	return standard_dev

def compute_entropy(input_data):
	entropy = []
	for i in range(0,15):
		freq = np.abs(np.fft.rfft(input_data[:,i]))
		entropy.append(stats.entropy(freq,base =2))
	return entropy

def compute_energy(input_data):
	energy = []
	for i in range(0,15):
		freq = np.abs(np.fft.rfft(input_data[:,i]))
		energy.append(np.sum(freq**2)/len(freq))
	return energy

def extract_features(input_data):
	X = []
	X.extend(compute_min(input_data))
	X.extend(compute_max(input_data))
	X.extend(compute_mean(input_data))
	X.extend(compute_energy(input_data))
	X.extend(compute_standard_dev(input_data))
	X.extend(compute_entropy(input_data))
	return X
