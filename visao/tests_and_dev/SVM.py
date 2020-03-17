import csv
import numpy
import pickle
from sklearn.svm import SVC
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report,confusion_matrix

input_data = []
target = []

with open('./../data_files/collected_data/all_statistics.csv', newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
    for (i, row) in enumerate(spamreader):
        if i > 1:
            input_data.append([int(i) for i in row[:-1]])
            target.append(int(row[-1]))

# split input_data in training and validation
input_data_train, input_data_validation, target_train, target_validation = train_test_split(input_data,target,test_size=0.2)

classifier = SVC(kernel='rbf')
classifier.fit(input_data_train,target_train)

target_pred = classifier.predict(input_data_validation)

print ("rbf (gausian)")
print (confusion_matrix(target_validation,target_pred))
print (classification_report(target_validation,target_pred))
print ("#"*60)

# save the classifier
with open('./../data_files/gausian_svm.pkl', 'wb') as fid:
    pickle.dump(classifier, fid)

# load it again using:
# with open('my_dumped_classifier.pkl', 'rb') as fid:
#     gnb_loaded = cPickle.load(fid)
