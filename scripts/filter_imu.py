#!/usr/bin/env python
import numpy as np
from numpy.linalg import norm
from geometry_msgs.msg import Quaternion
import serial
import rospy
import tf
import time
from sensor_msgs.msg import Imu
import pprint
import sys

np.set_printoptions(precision=3, linewidth=100)

def hat(w):
    return np.matrix([[0, -w[2], w[1]],[w[2], 0, -w[0]],[-w[1], w[0], 0]])

class IMU_estimator(object):
    def __init__(self):
        self.quat = np.array([0,0,0,1])
        self.dq = np.array([0.01,0.01,0.01,0.01])
        self.dt = 1./70
        # theta x y z
        n_state = 12
        n_sens = 3

        self.x = np.matrix(np.zeros(shape=(n_state, 1)))
        self.P = np.matrix(np.identity(n_state)*10)
        self.F = np.matrix(np.identity(n_state))

        self.F[0, 3] = self.dt
        self.F[1, 4] = self.dt
        self.F[2, 5] = self.dt
        self.F[6, -3] = self.dt
        self.F[7, -2] = self.dt
        self.F[8, -1] = self.dt

        self.H = np.matrix(np.zeros(shape=(n_sens, n_state)))
        self.H[-3, -3] = 1
        self.H[-2, -2] = 1
        self.H[-1, -1] = 1
        R_std = 0.1
        self.R = np.matrix(np.identity(n_sens)*R_std**2)
        self.I = np.matrix(np.identity(n_state))

    def kf_predict(self):
        self.x = self.F*self.x
        self.P = self.F * self.P * self.F.getT()

    def kf_correct(self, Z):
        w = Z - self.H * self.x
        S = self.H * self.P * self.H.getT() + self.R
        K = self.P * self.H.getT() * S.getI()
        self.x = self.x + K * w
        self.P = (self.I - K * self.H) * self.P

    def predict(self, w):
        # quat = self.quat + self.dt * w
        wx = w[1]
        wy = w[0]
        wz = w[2]

        q0_ = self.quat[0]
        q1_ = self.quat[1]
        q2_ = self.quat[2]
        q3_ = self.quat[3]
        dt = self.dt
        q0_pred = q0_ + 0.5*dt*( wx*q1_ + wy*q2_ + wz*q3_);
        q1_pred = q1_ + 0.5*dt*(-wx*q0_ - wy*q3_ + wz*q2_);
        q2_pred = q2_ + 0.5*dt*( wx*q3_ - wy*q0_ - wz*q1_);
        q3_pred = q3_ + 0.5*dt*(-wx*q2_ + wy*q1_ - wz*q0_);
        quat = [q0_pred, q1_pred, q2_pred, q3_pred]
        self.quat = self.quat_norm(quat)
        pass

    def quat_norm(self, quat):
        return quat / norm(quat)

    def correct(self, acc):
        x = acc[1]
        y = acc[0]
        z = acc[2]

        q0 = self.quat[0]
        q1 = self.quat[1]
        q2 = self.quat[2]
        q3 = self.quat[3]

        dq0 = self.dq[0]
        dq2 = self.dq[2]

        gx = (q0*q0 + q1*q1 - q2*q2 - q3*q3)*x + 2*(q1*q2 - q0*q3)*y + 2*(q1*q3 + q0*q2)*z
        gy = 2*(q1*q2 + q0*q3)*x + (q0*q0 - q1*q1 + q2*q2 - q3*q3)*y + 2*(q2*q3 - q0*q1)*z
        gz = 2*(q1*q3 - q0*q2)*x + 2*(q2*q3 + q0*q1)*y + (q0*q0 - q1*q1 - q2*q2 + q3*q3)*z
        # Delta quaternion that rotates the predicted gravity into the real gravity:
        self.dq[0] =  np.sqrt((gz + 1.) * 0.5);
        self.dq[1] = -gy/(2.0 * dq0);
        self.dq[2] =  gx/(2.0 * dq0);
        self.dq[3] =  0.0;
        # print self.dq

        # rot = np.array([-0.707, 0, 0, 0.707])
        # quat = self.dq
        # qv = quat[:-1]
        # pv = rot[:-1]
        # qw = quat[-1]
        # pw = rot[-1]
        # xyz = pw * qv + qw * pv + np.cross(pv, qv)
        # self.dq = np.array([xyz[0],xyz[1],xyz[2], pw*qw - np.inner(pv, qv)])


        self.scaleQuaternion(0.1)
        self.quatMult()
        pass

    def imu_callback(self, msg):
        self.predict(msg.angular_velocity)
        self.correct(msg.linear_acceleration)
        # scaleQuaternion(gain, dq0_acc, dq1_acc, dq2_acc, dq3_acc);
        # quaternionMultiplication(q0_pred, q1_pred, q2_pred, q3_pred,
        #                dq0_acc, dq1_acc, dq2_acc, dq3_acc,
        #                q0_, q1_, q2_, q3_);
        # normalizeQuaternion(q0_, q1_, q2_, q3_);

    def quatMult(self):
        # void quaternionMultiplication(
        #   double p0, double p1, double p2, double p3,
        #   double q0, double q1, double q2, double q3,
        #   double& r0, double& r1, double& r2, double& r3)
        # {
        # r = p q
        p0 = self.quat[0]
        p1 = self.quat[1]
        p2 = self.quat[2]
        p3 = self.quat[3]

        q0 = self.dq[0]
        q1 = self.dq[1]
        q2 = self.dq[2]
        q3 = self.dq[3]

        self.quat[0] = p0*q0 - p1*q1 - p2*q2 - p3*q3;
        self.quat[1] = p0*q1 + p1*q0 + p2*q3 - p3*q2;
        self.quat[2] = p0*q2 - p1*q3 + p2*q0 + p3*q1;
        self.quat[3] = p0*q3 + p1*q2 - p2*q1 + p3*q0;

        self.quat = self.quat_norm(self.quat)
        pass

    def scaleQuaternion(self, gain):
        dq0 = self.dq[0]
        dq1 = self.dq[1]
        dq2 = self.dq[2]
        dq3 = self.dq[3]

        if (dq0 < 0.0):
            #Slerp (Spherical linear interpolation):
            angle = np.arccos(dq0);
            A = np.sin(angle*(1.0 - gain))/np.sin(angle);
            B = np.sin(angle * gain)/np.sin(angle);
            dq0 = A + B * dq0;
            dq1 = B * dq1;
            dq2 = B * dq2;
            dq3 = B * dq3;
        else:
            # Lerp (Linear interpolation):
            dq0 = (1.0 - gain) + gain * dq0;
            dq1 = gain * dq1;
            dq2 = gain * dq2;
            dq3 = gain * dq3;
        self.dq = np.array([dq0, dq1, dq2, dq3])
        self.dq = self.quat_norm(self.dq)


# get IMU
# compute bias
# Predict from w
# Correct from acc



if __name__ == '__main__':
    rospy.init_node('tf_imu')
    br = tf.TransformBroadcaster()
    pub = rospy.Publisher('imu', Imu, queue_size=1)
    imuMsg = Imu()

    w = np.array([1, 2, 3]) # rotation rate vector
    print(hat(w))
    esti = IMU_estimator()

    port='/dev/ttyACM0'
    baudrate = 115200
    ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)
    ser.flushInput()
    ser.flushOutput()

    calib_array = []
    rospy.sleep(0.5)
    # for i in range(50):
    #     line = ser.readline()

    for i in range(2):
        line = ser.readline()
        words = line.strip().split(',')
        data = np.array(words).astype(np.float)
        calib_array.append(data[1:10])
    calib_array = np.array(calib_array)
    print calib_array.shape
    bias = calib_array.mean(axis=0)
    print bias
    print "Calibraiton complete"
    imu_gyro_prev = np.zeros(3)

    Z = np.matrix(np.zeros(shape=(3, 1)))

    while not rospy.is_shutdown():
        start = time.time()

        line = ser.readline()
        words = line.strip().split(',')
        data = np.array(words).astype(np.float)

        deg2rad = np.pi/180.

        imu_raw = data[1:11]
        imu_acc = imu_raw[:3]
        imu_gyro = (imu_raw[3:6] - bias[3:6]) * deg2rad
        # imu_mag = imu_raw[6:]
        # print imu_gyro
        imuMsg.angular_velocity.x = imu_gyro[1]
        imuMsg.angular_velocity.y = imu_gyro[0]
        imuMsg.angular_velocity.z = -imu_gyro[2]

        Z[-3] = imu_gyro[1]
        Z[-2] = imu_gyro[0]
        Z[-1] = -imu_gyro[2]


        accel_factor = 1
        imuMsg.linear_acceleration.x = -imu_acc[2] * accel_factor
        imuMsg.linear_acceleration.y =  -imu_acc[0]* accel_factor
        imuMsg.linear_acceleration.z = imu_acc[1] * accel_factor
        # print imu_mag/ norm(imu_mag)
        # print('{}'.format(imu_gyro * deg2rad))
        if any((np.abs(imu_gyro - imu_gyro_prev)) > 10):
            print('{}'.format(imu_gyro))
            imu_gyro_prev = np.zeros(3)
            continue
        imu_gyro_prev = imu_gyro

        esti.predict(imu_gyro)
        # esti.predict(np.array([0.,0.,0.]))
        # print imu_raw[:3] / norm(imu_raw[:3])
        esti.correct(imu_acc / norm(imu_acc))

        esti.kf_correct(Z)
        esti.kf_predict()

        # print "<"
        # print imu_gyro
        # print esti.x[-6:].getT()


        q0, q1, q2, q3 = data[-4:]
        # q1 = data[-7]
        # q2 = data[-6]
        # q3 = data[-5]

        quat = np.array([q0, q1, q2, q3])
        quat = np.array(esti.quat)
        rot = np.array([0.707, -0.707, 0, 0])
        qv = quat[:-1]
        pv = rot[:-1]
        qw = quat[-1]
        pw = rot[-1]
        xyz = pw * qv + qw * pv + np.cross(pv, qv)
        # quat = np.array([xyz[0],xyz[1],xyz[2], pw*qw - np.inner(pv, qv)])

        # quat = np.array(esti.quat)
        # quat = tf.transformations.quaternion_from_euler(esti.x[6], esti.x[7], esti.x[8])
        quat = np.array([quat[1], quat[2], quat[3], quat[0]])
        br.sendTransform((0, 0, 0),
                    quat ,
                    #  tf.transformations.quaternion_from_euler(esti.x[6], esti.x[7], esti.x[8]),
                     rospy.Time.now(),
                     "imu",
                     "world")
        imuMsg.orientation.x = quat[0]
        imuMsg.orientation.y = quat[1]
        imuMsg.orientation.z = quat[2]
        imuMsg.orientation.w = quat[3]
        imuMsg.header.stamp= rospy.Time.now()
        imuMsg.header.frame_id = 'imu'
        pub.publish(imuMsg)
        # print 1./(time.time() - start)
    ser.close()
