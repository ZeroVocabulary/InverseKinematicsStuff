""" Proof of concept of Denavit Hartenberg parameter calibration. Example output:
Epoch: 0         loss: 7.651479244232178
Epoch: 1000      loss: 1.7052144585250062e-08
Epoch: 2000      loss: 8.029132914089132e-13
Epoch: 3000      loss: 5.027800398238469e-11
Epoch: 4000      loss: 4.175149115326349e-11
Epoch: 5000      loss: 1.446665010007564e-11
Epoch: 6000      loss: 3.842615114990622e-11
Epoch: 7000      loss: 3.092281986027956e-11
Epoch: 8000      loss: 5.834976946061943e-11
Epoch: 9000      loss: 6.230038707144558e-11
Real Lengths: 3.0, 3.0
Learned Lengths: 3.0000054836273193, 3.0000054836273193
"""


import numpy as np
import tensorflow as tf
import math

tf.reset_default_graph()

# Real length of links. Used for test, since we need to calculate the end effector position.
# Normally the end effector position would be measured IRL and we wouldn't know this.
lenLink1Truth = tf.constant(3.0)
lenLink2Truth = tf.constant(3.0)

# Generate random angles.
# Normally the angles would be sensed IRL from encoders.
# This is also an "infinite" dataset I guess because its randomly generated every time.
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

y_true = tf.stack([posXTruth, posYTruth],1)
y_pred = tf.stack([posX, posY],1)
loss = tf.losses.mean_squared_error(labels=y_true, predictions=y_pred)

optimizer = tf.train.GradientDescentOptimizer(0.01)
#optimizer = tf.train.AdamOptimizer()
train = optimizer.minimize(loss)

sess = tf.Session()
init = tf.global_variables_initializer()
sess.run(init)

for i in range(10000):
    _, loss_value = sess.run((train, loss))
    if i % 1000 == 0:
        print("Epoch: {1}\t loss: {0}".format(loss_value, i))

print("Real Lengths: {0}, {1}".format(sess.run(lenLink1Truth), sess.run(lenLink2Truth)))
print("Learned Lengths: {0}, {1}".format(sess.run(lenLink1), sess.run(lenLink2)))

'''
writer = tf.summary.FileWriter('.')
writer.add_graph(tf.get_default_graph())
writer.flush()
'''
