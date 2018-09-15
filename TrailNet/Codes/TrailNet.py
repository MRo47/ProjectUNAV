from keras.models import Sequential
from keras.layers import Convolution2D
from keras.layers import MaxPooling2D
from keras.layers import Flatten
from keras.layers import Dense
import json


#initialise CNN
classifier = Sequential()

#Convolution L1
classifier.add(Convolution2D(32,4,4,input_shape = (101,101,3),activation = 'relu'))

#maxpooling L2
classifier.add(MaxPooling2D(pool_size=(2,2)))

#Convolution L3
classifier.add(Convolution2D(32,4,4,activation = 'relu')) #keras figures out automatically the size coming from maxpooling

#maxpooling L4
classifier.add(MaxPooling2D(pool_size=(2,2)))

#Convolution L5
classifier.add(Convolution2D(32,4,4,activation = 'relu')) #keras figures out automatically the size coming from maxpooling

#maxpooling L6
classifier.add(MaxPooling2D(pool_size=(2,2)))

#Convolution L7
classifier.add(Convolution2D(32,4,4,activation = 'relu')) #keras figures out automatically the size coming from maxpooling

#maxpooling L8
classifier.add(MaxPooling2D(pool_size=(2,2)))

#flattenning L9
classifier.add(Flatten())

#full connection
classifier.add(Dense(units = 200, activation = 'relu'))

#output layer
classifier.add(Dense(units = 3, activation = 'softmax'))  #to get probability output

#comipling Cnn
classifier.compile(optimizer = 'adam', loss = 'categorical_crossentropy', metrics = ['accuracy'])

#fitting CNN to images
from keras.preprocessing.image import ImageDataGenerator

#training data augmentation object
train_datagen = ImageDataGenerator(
        rescale=1./255,
        shear_range=0.2,
        zoom_range=0.2,
        horizontal_flip=False)

#test data augmentation object
test_datagen = ImageDataGenerator(rescale=1./255)

#directory structure
#dataset
#  |__training set
#  |     |__category1
#  |     |__category2
#  |     |_........
#  |__test set
#  |     |__category1
#  |     |__category2
#  |     |_........

#use objects to generate training and test sets
training_set = train_datagen.flow_from_directory(
        'dataset/training',
        target_size=(101, 101),   #high res images for better op
        batch_size=32,
        class_mode='categorical')

test_set = test_datagen.flow_from_directory(
        'dataset/validation',
        target_size=(101, 101),
        batch_size=32,
        class_mode='categorical')

training_set.class_indices

model_json = classifier.to_json()
with open("TrailNet2.json", "w") as json_file:
    json_file.write(model_json)
print("Saved model to disk")

from keras.callbacks import ModelCheckpoint
rom keras.callbacks import TensorBoard
from keras.callbacks import CSVLogger
from keras.callbacks import EarlyStopping
checkpointer = ModelCheckpoint(filepath='TrailNetWeights2.hdf5', verbose=1, save_best_only=True)
hist_pointer = TensorBoard(log_dir='./logs', batch_size=32, write_graph=True, write_images=True)
csv_logger = CSVLogger('logC.csv',separator=',',append=True)
E_stop = EarlyStopping(monitor='val_acc',min_delta=0.01,patience=1,verbose=1,mode='auto')

histK = classifier.fit_generator(training_set,
                                steps_per_epoch=27000, #training images
                                epochs=30,
                                validation_data=test_set,
                                validation_steps=13500, #test images
                                callbacks = [checkpointer,hist_pointer,csv_logger,E_stop])

histKDict = histK.history
json.dump(histKDict,open('hist.json','w'))
