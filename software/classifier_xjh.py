import pandas as pd
import numpy as np

from keras.models import Sequential
from keras.layers import Dense
from keras.wrappers.scikit_learn import KerasClassifier

from sklearn import preprocessing
from sklearn.model_selection import train_test_split
from sklearn.metrics import confusion_matrix, accuracy_score
from sklearn.externals import joblib
from sklearn.model_selection import StratifiedKFold

def fully_connected_model():
    model = Sequential()

    model.add(Dense(128, input_dim=90, activation='relu'))
    model.add(Dense(64, activation='relu'))
    model.add(Dense(64, activation='relu'))
    model.add(Dense(64, activation='relu'))
    model.add(Dense(32, activation='relu'))
    model.add(Dense(14, activation='softmax')) 

    model.compile(loss='sparse_categorical_crossentropy', optimizer='Adam', metrics=['accuracy'])

    return model

def k_fold_cross_validate(X, y, model_path, scaler_path):
    scaler = joblib.load(scaler_path)
    model = joblib.load(model_path)
    
    seed = 7
    np.random.seed(seed)
    cvscores = []
    kfold = StratifiedKFold(n_splits=10, shuffle=True, random_state=seed)
    
    for train, test in kfold.split(X, y):
        scores = model.evaluate(X[test], y[test], verbose=0)
        print("%s: %.2f%%" % (model.metrics_names[1], scores[1]*100))
        cvscores.append(scores*100)

    print("%.2f%% (+/- %.2f%%)" % (np.mean(cvscores), np.std(cvscores)))


dataframe = pd.read_csv('final.csv', header=None)
dataset = dataframe.values
len_data = len(dataset[0])
X = dataset[:, 0:len_data-1].astype(float)
y = dataset[:, len_data-1].astype(int)

scaler = preprocessing.StandardScaler()
scaler.fit(X) 
X = np.nan_to_num(scaler.transform(X))
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.5, random_state=4)

estimator = KerasClassifier(build_fn=fully_connected_model, epochs=200, batch_size=100, verbose=1)
estimator.fit(X_train, y_train)
joblib.dump(estimator.model, 'model/model.joblib')
joblib.dump(scaler, 'model/scaler.joblib')

k_fold_cross_validate(X_test, y_test, 'model/model.joblib', 'model/scaler.joblib')
