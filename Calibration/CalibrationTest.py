""" Proof of concept of Denavit Hartenberg parameter calibration. Example output:
Epoch: 0         loss: 6.163139343261719         lenLink1: 4.969184398651123     lenLink2: 4.969184398651123
Epoch: 1000      loss: 2.972727841665801e-09     lenLink1: 3.0001275539398193    lenLink2: 3.0001275539398193
Epoch: 2000      loss: 5.4576787533733295e-11    lenLink1: 3.0000057220458984    lenLink2: 3.0000054836273193
Epoch: 3000      loss: 2.9473312679328956e-11    lenLink1: 3.0000057220458984    lenLink2: 3.0000054836273193
Epoch: 4000      loss: 1.1972645097557688e-12    lenLink1: 3.0000057220458984    lenLink2: 3.0000054836273193
Epoch: 5000      loss: 5.6076032706187107e-11    lenLink1: 3.0000057220458984    lenLink2: 3.0000054836273193
Epoch: 6000      loss: 3.155520289510605e-11     lenLink1: 3.0000057220458984    lenLink2: 3.0000054836273193
Epoch: 7000      loss: 7.270628543665225e-12     lenLink1: 3.0000057220458984    lenLink2: 3.0000054836273193
Epoch: 8000      loss: 2.802380549837835e-11     lenLink1: 3.0000057220458984    lenLink2: 3.0000054836273193
Epoch: 9000      loss: 4.311573320592288e-11     lenLink1: 3.0000057220458984    lenLink2: 3.0000054836273193
Real Lengths: 3.0, 3.0
Learned Lengths: 3.0000057220458984, 3.0000054836273193
"""


import numpy as np
import tensorflow as tf
import math

tf.reset_default_graph()

# Real length of links. Used for test, since we need to calculate the end effector position.
# Normally the end effector position would be measured IRL and we wouldn't know this.
lenLink1Truth = tf.constant(3.0)
lenLink2Truth = tf.constant(3.0)

# Generate random angles (new angles every sess.run)
# Normally the angles would be sensed IRL from encoders.
# This is also an "infinite" dataset because its randomly generated every time.
angle1 = tf.random_uniform([1], minval=0, maxval=2*math.pi)
angle2 = tf.random_uniform([1], minval=0, maxval=2*math.pi)

# forward kinematics to generate the truth values from the previously randomly generated angles
# Normally would be measured (end effector position)
posXTruth = lenLink1Truth*tf.cos(angle1) + lenLink2Truth*tf.cos(angle1 + angle2)
posYTruth = lenLink1Truth*tf.sin(angle1) + lenLink2Truth*tf.sin(angle1 + angle2)

# Length of the links. We will be calibrating this. It starts at 5 for no particular reason
lenLink1 = tf.get_variable("lenLink1", initializer=tf.constant(5.0))
lenLink2 = tf.get_variable("lenLink2", initializer=tf.constant(5.0))

# forward kinematics
posX = lenLink1*tf.cos(angle1) + lenLink2*tf.cos(angle1 + angle2)
posY = lenLink1*tf.sin(angle1) + lenLink2*tf.sin(angle1 + angle2)

# Create the predicted and true arrays. The arrays give position in x and y.
y_pred = tf.stack([posX, posY],1)
y_true = tf.stack([posXTruth, posYTruth],1)

# Define loss to be the mean squared error between the predicted position and true position
loss = tf.losses.mean_squared_error(labels=y_true, predictions=y_pred)

# Create the gradient descent optimizer
optimizer = tf.train.GradientDescentOptimizer(0.01)
train = optimizer.minimize(loss)

sess = tf.Session()
init = tf.global_variables_initializer()
sess.run(init)

for i in range(10000):
    _, loss_value, lenLink1_value, lenLink2_value = sess.run((train, loss, lenLink1, lenLink2))
    if i % 1000 == 0:
        print("Epoch: {1}\t loss: {0}\t lenLink1: {2}\t lenLink2: {3}".format(loss_value, i, lenLink1_value, lenLink2_value))
        #print("Epoch: {1}\t lenLink1: {2}\t lenLink2: {3}".format(loss_value, i, lenLink1_value, lenLink2_value))

print("Real Lengths: {0}, {1}".format(sess.run(lenLink1Truth), sess.run(lenLink2Truth)))
print("Learned Lengths: {0}, {1}".format(sess.run(lenLink1), sess.run(lenLink2)))
