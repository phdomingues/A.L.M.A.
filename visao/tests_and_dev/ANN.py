import csv
import numpy as np
import pickle
from sklearn.model_selection import train_test_split
import tensorflow as tf
from tensorflow.python.keras.callbacks import TensorBoard

# Load data
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

input_data_train = np.array([np.array(i) for i in input_data_train])
target_train = np.array([np.array(i) for i in target_train])
input_data_validation = np.array([np.array(i) for i in input_data_validation])
target_validation = np.array([np.array(i) for i in target_validation])

# logs
tensorboard = TensorBoard(log_dir='./tb')

# defining the network model
model = tf.keras.models.Sequential()
model.add(tf.keras.layers.Dense(12, input_dim=16, activation='relu'))
model.add(tf.keras.layers.Dense(8, activation='relu'))
model.add(tf.keras.layers.Dense(1, activation='sigmoid'))

# compile model
model.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy'])

# fit
model.fit(input_data_train, target_train, epochs=150, batch_size=10, validation_data=(input_data_validation, target_validation), callbacks=[tensorboard])


# >>> save model <<< #
model.save("model.h5")
