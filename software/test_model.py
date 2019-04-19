import pandas as pd
from os import listdir
from classifier import Classifier

clf = Classifier('model/model.joblib', 'model/scaler.joblib')
num = 0
correct = 0
for f in listdir("raw_data"):
    if ".txt" in f:
        print(f)
        dataframe = pd.read_csv("raw_data/"+f, sep=",", header=None)
        dataset = dataframe.values
        i = 0
        while i+50 <= len(dataset): 
            num += 1
            X = dataset[i:i+50,0:15]
            prediction = clf.predict(X)
            if "chicken" in f:
                if prediction==0:
                    correct += 1
            if "cowboy_front" in f:
                if prediction==1:
                    correct += 1
            if "cowboy_back" in f:
                if prediction==2:
                    correct += 1
            if "crab" in f:
                if prediction==3:
                    correct += 1
            if "hunchback" in f:
                if prediction==4:
                    correct += 1
            if "raffles_left" in f:
                if prediction==5:
                    correct += 1
            if "raffles_right" in f:
                if prediction==6:
                    correct += 1
            if "running" in f:
                if prediction==7:
                    correct += 1
            if "james" in f:
                if prediction==8:
                    correct += 1
            if "snake" in f:
                if prediction==9:
                    correct += 1
            if "double" in f:
                if prediction==10:
                    correct += 1
            if "mermaid" in f:
                if prediction==11:
                    correct += 1
            if "final" in f:
                if prediction==12:
                    correct += 1
            i += 10
        print('total: ', num, ' correct: ', correct, ' accrarcy:' , correct/num)
