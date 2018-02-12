#!/usr/bin/env python

import sys
import os
import rospy
import rospkg

from threading import Thread

from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding.QtGui import QWidget
from trainergui import Ui_MainWindow

from inmoov_msgs.msg import MotorStatus
from inmoov_msgs.msg import MotorCommand
from inmoov_msgs.srv import MotorParameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# https://github.com/ColinDuquesnoy/QDarkStyleSheet
import qdarkstyle

# https://nikolak.com/pyqt-qt-designer-getting-started/
class ExampleApp(QtGui.QMainWindow, Ui_MainWindow):
    def __init__(self):
        # Explaining super is out of the scope of this article
        # So please google it if you're not familar with it
        # Simple reason why we use it here is that it allows us to
        # access variables, methods etc in the design.py file
        super(self.__class__, self).__init__()
        self.setupUi(self)  # This is defined in design.py file automatically
        # It sets up layout and widgets that are defined

        self.parameterTopic = ["servobus/torso/motorparameter","servobus/leftarm/motorparameter","servobus/rightarm/motorparameter"]

        self.motorcommand = MotorCommand()
        self.jointcommand = JointState()
        
        self.jointNames = []
        
        for servo in range (0, 11):
            self.jointNames.append( rospy.get_param('servobus/torso/servomap/'+str(servo)+'/name'))
            
        for servo in range (0, 11):
            self.jointNames.append( rospy.get_param('servobus/leftarm/servomap/'+str(servo)+'/name'))
            
        for servo in range (0, 11):
            self.jointNames.append( rospy.get_param('servobus/rightarm/servomap/'+str(servo)+'/name'))
        
        #print(self.jointNames)
        
        
            #'right_pinky','right_ring','right_middle','right_index','right_thumb',
            #'right_hand','right_bicep','right_bicep_rotate','right_shoulder_side','right_shoulder_up','','',
            
            #'eye_leftright','eyes_updown','jaw','head_leftright','head_updown','head_tilt','waist_lean','waist_rotate','','','','',  
            
            #'left_pinky','left_ring','left_middle','left_index','left_thumb',
            #'left_hand','left_bicep','left_bicep_rotate','left_shoulder_side','left_shoulder_up','','',
            


        self.setupDropDowns()
        
        self.cmbBus.currentIndexChanged.connect(self.busChanged)
        self.cmbServo.currentIndexChanged.connect(self.servoChanged)

        self.txtGoal.editingFinished.connect(self.setGoal)
        self.txtMinPulse.editingFinished.connect(self.setMinPulse)
        self.txtMaxPulse.editingFinished.connect(self.setMaxPulse)
        self.txtMinGoal.editingFinished.connect(self.setMinGoal)
        self.txtMaxGoal.editingFinished.connect(self.setMaxGoal)
        self.txtMinSensor.editingFinished.connect(self.setMinSensor)
        self.txtMaxSensor.editingFinished.connect(self.setMaxSensor)
        self.chkEnabled.stateChanged.connect(self.setEnabled)
        self.chkCalibrated.stateChanged.connect(self.setCalibrated)
        
        self.sliderGoal.valueChanged.connect(self.sliderChanged)

        rospy.init_node('trainer', anonymous=True)
        
        self.commandPublisher = []
        self.commandPublisher.append(rospy.Publisher("servobus/torso/motorcommand", MotorCommand, queue_size=10))
        self.commandPublisher.append(rospy.Publisher("servobus/leftarm/motorcommand", MotorCommand, queue_size=10))
        self.commandPublisher.append(rospy.Publisher("servobus/rightarm/motorcommand", MotorCommand, queue_size=10))
        
        self.statusSubscriber = []
        self.statusSubscriber.append(rospy.Subscriber("servobus/torso/motorstatus", MotorStatus, self.callback0))
        self.statusSubscriber.append(rospy.Subscriber("servobus/leftarm/motorstatus", MotorStatus, self.callback1))
        self.statusSubscriber.append(rospy.Subscriber("servobus/rightarm/motorstatus", MotorStatus, self.callback2))
        
        self.jointPublisher = rospy.Publisher("joint_command", JointState, queue_size=10)
        
        self.bus = 0
        self.servo = 0
        
        self.busChanged()
        self.servoChanged()
        
        

    def busChanged(self):
        # unregister topics and reregister to the new ones
        self.bus = self.cmbBus.currentIndex()
        
        self.cmbServo.clear()
        
        for s in range(0, 11):
            self.cmbServo.addItem(self.jointNames[(self.bus * 11) + s])
        
        #self.commandPublisher.unregister()
        #self.commandPublisher = rospy.Publisher(self.commandTopic[bus], MotorCommand, queue_size=10)
        #self.statusSubscriber.unregister()
        #self.statusSubscriber = rospy.Subscriber(self.statusTopic[self.bus], MotorStatus, self.callback)

        
        self.servoChanged()
        
    def servoChanged(self):
    
        if self.cmbServo.count() > 0:
            self.servo = self.cmbServo.currentIndex()
        
            self.getMinPulse()
            self.getMaxPulse()
            self.getMinGoal()
            self.getMaxGoal()
            self.getGoal()
            self.getMinSensor()
            self.getMaxSensor()
            self.getEnabled()
            self.getCalibrated()
    
    def callback0(self, data):
        if data.id == self.servo and self.bus == 0:
            #print data.posraw
            #self.chkEnabled.setChecked(bool(data.enabled))
            self.txtPosition.setText(str(data.position))
            self.txtSpeed.setText(str(data.presentspeed))
            self.txtSensorRaw.setText(str(data.posraw))
            self.chkMoving.setChecked(bool(data.moving))
            self.chkPower.setChecked(bool(data.power))
            #self.txtGoal.setText(str(data.goal))
            
    def callback1(self, data):
        if data.id == self.servo and self.bus == 1:
            #print data.posraw
            #self.chkEnabled.setChecked(bool(data.enabled))
            self.txtPosition.setText(str(data.position))
            self.txtSpeed.setText(str(data.presentspeed))
            self.txtSensorRaw.setText(str(data.posraw))
            self.chkMoving.setChecked(bool(data.moving))
            self.chkPower.setChecked(bool(data.power))
            #self.txtGoal.setText(str(data.goal))

    def callback2(self, data):
        if data.id == self.servo and self.bus == 2:
            #print data.posraw
            #self.chkEnabled.setChecked(bool(data.enabled))
            self.txtPosition.setText(str(data.position))
            self.txtSpeed.setText(str(data.presentspeed))
            self.txtSensorRaw.setText(str(data.posraw))
            self.chkMoving.setChecked(bool(data.moving))
            self.chkPower.setChecked(bool(data.power))
            #self.txtGoal.setText(str(data.goal))
    
    def degreestoradians(self, d):
        return d*(3.1415926/180.0)
    
    def setupDropDowns(self):
    
        self.cmbBus.addItem(rospy.get_param('/servobus/torso/name'))
        self.cmbBus.addItem(rospy.get_param('/servobus/leftarm/name'))
        self.cmbBus.addItem(rospy.get_param('/servobus/rightarm/name'))

        for servo in range (0, 11):
            print('/servobus/torso/servomap/' + str(servo) + '/name')
            self.cmbServo.addItem(rospy.get_param('/servobus/torso/servomap/' + str(servo) + '/name'))

        #self.cmbServo.addItem('Servo 00')
        #self.cmbServo.addItem('Servo 01')
        #self.cmbServo.addItem('Servo 02')
        #self.cmbServo.addItem('Servo 03')
        #self.cmbServo.addItem('Servo 04')
        #self.cmbServo.addItem('Servo 05')
        #self.cmbServo.addItem('Servo 06')
        #self.cmbServo.addItem('Servo 07')
        #self.cmbServo.addItem('Servo 08')
        #self.cmbServo.addItem('Servo 09')
        #self.cmbServo.addItem('Servo 10')
        #self.cmbServo.addItem('Servo 11')

        self.cmbSmoothing.addItem('0 - Instant')
        self.cmbSmoothing.addItem('1 - Max Speed')
        self.cmbSmoothing.addItem('2 - Linear Ramp')
        self.cmbSmoothing.addItem('3 - COS Ramp')
        self.cmbSmoothing.addItem('4 - COS^2 Ramp')
        
    def sliderChanged(self, i):
        self.txtGoal.setText(str(i/1000.0))
        self.setGoal()
    
    def setGoal(self):
        #print(str(value))
        self.motorcommand.id = self.cmbServo.currentIndex()
        self.motorcommand.parameter = 0x1E
        self.motorcommand.value = float(self.txtGoal.text())
        #print(self.motorcommand.value)
        self.commandPublisher[self.bus].publish(self.motorcommand)
        
        self.jointcommand.header = Header()
        self.jointcommand.header.stamp = rospy.Time.now()
        self.jointcommand.name = [self.jointNames[((self.bus * 12) + self.servo)]]
        self.jointcommand.position = [self.degreestoradians(float(self.txtGoal.text()))]
        self.jointcommand.velocity = []
        self.jointcommand.effort = []
        self.jointPublisher.publish(self.jointcommand)
        
    
    def getGoal(self):
        bus = self.cmbBus.currentIndex()
        motorparameter = rospy.ServiceProxy(self.parameterTopic[bus], MotorParameter)
        value = motorparameter(self.cmbServo.currentIndex(), 0x1E).data
        self.txtGoal.setText(str(value))
        self.sliderGoal.setValue(int(value * 1000.0))
       
    def setMinPulse(self):
        #print(str(value))
        self.motorcommand.id = self.cmbServo.currentIndex()
        self.motorcommand.parameter = 0x14
        self.motorcommand.value = float(self.txtMinPulse.text())
        self.commandPublisher[self.bus].publish(self.motorcommand)
       
    def getMinPulse(self):
        bus = self.cmbBus.currentIndex()
        motorparameter = rospy.ServiceProxy(self.parameterTopic[bus], MotorParameter)
        self.txtMinPulse.setText(str(motorparameter(self.cmbServo.currentIndex(), 0x14).data))
    
    def setMaxPulse(self):
        #print(str(value))
        self.motorcommand.id = self.cmbServo.currentIndex()
        self.motorcommand.parameter = 0x16
        self.motorcommand.value = float(self.txtMaxPulse.text())
        self.commandPublisher[self.bus].publish(self.motorcommand)
       
    def getMaxPulse(self):
        bus = self.cmbBus.currentIndex()
        motorparameter = rospy.ServiceProxy(self.parameterTopic[bus], MotorParameter)
        self.txtMaxPulse.setText(str(motorparameter(self.cmbServo.currentIndex(), 0x16).data))
        
    def setMinGoal(self):
        #print(str(value))
        self.motorcommand.id = self.cmbServo.currentIndex()
        self.motorcommand.parameter = 0x06
        self.motorcommand.value = float(self.txtMinGoal.text())
        self.sliderGoal.setMinimum(int(self.motorcommand.value * 1000.0))
        self.commandPublisher[self.bus].publish(self.motorcommand)
       
    def getMinGoal(self):
        bus = self.cmbBus.currentIndex()
        motorparameter = rospy.ServiceProxy(self.parameterTopic[bus], MotorParameter)
        value = motorparameter(self.cmbServo.currentIndex(), 0x06).data
        self.txtMinGoal.setText(str(value))
        self.sliderGoal.setMinimum(int(value * 1000.0))
    
    def setMaxGoal(self):
        #print(str(value))
        self.motorcommand.id = self.cmbServo.currentIndex()
        self.motorcommand.parameter = 0x08
        self.motorcommand.value = float(self.txtMaxGoal.text())
        self.sliderGoal.setMaximum(int(self.motorcommand.value * 1000.0))
        self.commandPublisher[self.bus].publish(self.motorcommand)
       
    def getMaxGoal(self):
        bus = self.cmbBus.currentIndex()
        motorparameter = rospy.ServiceProxy(self.parameterTopic[bus], MotorParameter)
        value = motorparameter(self.cmbServo.currentIndex(), 0x08).data
        self.txtMaxGoal.setText(str(value))
        self.sliderGoal.setMaximum(int(value * 1000.0))
        
    def setMinSensor(self):
        #print(str(value))
        self.motorcommand.id = self.cmbServo.currentIndex()
        self.motorcommand.parameter = 0xA2
        self.motorcommand.value = float(self.txtMinSensor.text())
        self.commandPublisher[self.bus].publish(self.motorcommand)
       
    def getMinSensor(self):
        bus = self.cmbBus.currentIndex()
        motorparameter = rospy.ServiceProxy(self.parameterTopic[bus], MotorParameter)
        self.txtMinSensor.setText(str(motorparameter(self.cmbServo.currentIndex(), 0xA2).data))
    
    def setMaxSensor(self):
        #print(str(value))
        self.motorcommand.id = self.cmbServo.currentIndex()
        self.motorcommand.parameter = 0xA4
        self.motorcommand.value = float(self.txtMaxSensor.text())
        self.commandPublisher[self.bus].publish(self.motorcommand)
       
    def getMaxSensor(self):
        bus = self.cmbBus.currentIndex()
        motorparameter = rospy.ServiceProxy(self.parameterTopic[bus], MotorParameter)
        self.txtMaxSensor.setText(str(motorparameter(self.cmbServo.currentIndex(), 0xA4).data))
        
    def setEnabled(self):
        self.motorcommand.id = self.cmbServo.currentIndex()
        self.motorcommand.parameter = 0x18
        self.motorcommand.value = float(self.chkEnabled.isChecked())
        self.commandPublisher[self.bus].publish(self.motorcommand)
    
    def getEnabled(self):
        bus = self.cmbBus.currentIndex()
        motorparameter = rospy.ServiceProxy(self.parameterTopic[bus], MotorParameter)
        self.chkEnabled.setChecked(bool(motorparameter(self.cmbServo.currentIndex(), 0x18).data))
        
    def setCalibrated(self):
        self.motorcommand.id = self.cmbServo.currentIndex()
        self.motorcommand.parameter = 0xA0
        self.motorcommand.value = float(self.chkCalibrated.isChecked())
        self.commandPublisher[self.bus].publish(self.motorcommand)
    
    def getCalibrated(self):
        bus = self.cmbBus.currentIndex()
        motorparameter = rospy.ServiceProxy(self.parameterTopic[bus], MotorParameter)
        self.chkCalibrated.setChecked(bool(motorparameter(self.cmbServo.currentIndex(), 0xA0).data))

def main():
    app = QtGui.QApplication(sys.argv)  # A new instance of QApplication
    app.setStyleSheet(qdarkstyle.load_stylesheet(pyside=False))
    form = ExampleApp()  # We set the form to be our ExampleApp (design)
    form.show()  # Show the form
    app.exec_()  # and execute the app
   
if __name__ == '__main__':  # if we're running file directly and not importing it
    main()
