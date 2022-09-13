import csv
import numpy as np
import cv2
import matplotlib.pyplot as plt
import os
import sklearn

from urllib.request import urlretrieve
from sklearn.preprocessing import LabelBinarizer
from zipfile import ZipFile

from keras.models import Sequential
from keras.layers.core import Dense, Activation, Flatten, Dropout
from keras.layers.convolutional import Conv2D
from keras.layers import MaxPooling2D, Cropping2D, Lambda

from sklearn.utils import shuffle
from sklearn.model_selection import train_test_split

def download(url,file):
    if not os.path.isfile(file):
        print("Download file... " + file + " ...")
        urlretrieve(url,file)
        print("File downloaded")
    
download('https://s3.amazonaws.com/video.udacity-data.com/topher/2016/December/584f6edd_data/data.zip','data.zip')
print("All the files are downloaded")

def uncompress_features_labels(dir,name):
    if(os.path.isdir(name)):
        print('Data extracted')
    else:
        with ZipFile(dir) as zipf:
            zipf.extractall('data')
uncompress_features_labels('data.zip','data')

def data_Files(mypath):
    onlyfiles = [f for f in os.listdir(mypath) if os.path.isfile(os.path.join(mypath, f))]
    print(onlyfiles)
    
print('All files downloaded and extracted')

lines=[]
with open('./data/data/driving_log.csv') as csvfile:
    reader=csv.reader(csvfile)
    next(reader, None)
    for line in reader:
        lines.append(line)
    
train_samples, validation_samples = train_test_split(lines,test_size=0.15)
    
def generator(samples, batch_size=32):
    num_samples = len(samples)
   
    while 1: 
        shuffle(samples) 
        for offset in range(0, num_samples, batch_size):
            
            batch_samples = samples[offset:offset+batch_size]

            images = []
            angles = []
            for batch_sample in batch_samples:
                    for i in range(0,3):
                        name = './data/data/IMG/'+batch_sample[i].split('/')[-1]
                        center_image = cv2.cvtColor(cv2.imread(name), cv2.COLOR_BGR2RGB)
                        center_angle = float(batch_sample[3]) 
                        images.append(center_image)
                        if(i==0):
                            angles.append(center_angle)
                        elif(i==1):
                            angles.append(center_angle+0.2)
                        elif(i==2):
                            angles.append(center_angle-0.2)
                        
                        images.append(cv2.flip(center_image,1))
                        if(i==0):
                            angles.append(center_angle*-1)
                        elif(i==1):
                            angles.append((center_angle+0.2)*-1)
                        elif(i==2):
                            angles.append((center_angle-0.2)*-1)
                        
        
            x_train = np.array(images)
            y_train = np.array(angles)
            
            yield sklearn.utils.shuffle(x_train, y_train)
    
train_generator = generator(train_samples, batch_size=32)
validation_generator = generator(validation_samples, batch_size=32)

model=Sequential()
model.add(Lambda(lambda x: (x/255.0)-0.5, input_shape=(160,320,3)))
model.add(Cropping2D(cropping=((70,25),(0,0))))

model.add(Conv2D(24,(5,5), strides=(2,2)))
model.add(Activation('relu'))

model.add(Conv2D(36,(5,5), strides=(2,2)))
model.add(Activation('relu'))
          
model.add(Conv2D(48,(5,5), strides=(2,2)))
model.add(Activation('relu'))
          
model.add(Conv2D(64,(3,3), strides=(2,2)))
model.add(Activation('relu'))
                    
model.add(Flatten())

model.add(Dense(100))
model.add(Activation('relu'))
          
model.add(Dropout(0.5))
          
model.add(Dense(50))
model.add(Activation('relu'))

model.add(Dense(10))
model.add(Activation('relu'))
          
model.add(Dense(1))
          
model.compile(loss='mse',optimizer='adam')

model.fit_generator(train_generator, samples_per_epoch=len(train_samples), validation_data=validation_generator, nb_val_samples=len(validation_samples), nb_epoch=5, verbose=1)
model.save('model.h5')
print("Model saved")
model.summary()