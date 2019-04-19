import numpy as np
from sklearn.externals import joblib
from feature_extract import extract_features

class Classifier:
    def __init__(self, model_path, scaler_path):
        print('Loading model from ' +  model_path)
        self.scaler = joblib.load(scaler_path)
        self.model = joblib.load(model_path)
        print('Successfully loaded the model: ', self.model)

    def predict(self, input_data):
        data = np.asarray(extract_features(np.asarray(input_data)))
        data = np.array([data])
        features = np.nan_to_num(self.scaler.transform(data))
        return self.model.predict_classes(features, verbose=0)
