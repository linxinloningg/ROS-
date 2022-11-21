from ui.ui import Ui_Form
from PyQt5 import QtWidgets, QtCore


class QtThread(QtCore.QThread):
    def __init__(self):
        super().__init__()

    def init(self, func, args):
        self.func = func
        self.args = args

    def run(self):
        try:
            self.func(*self.args)
        except Exception as _:
            print(_)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)

        self.ui = Ui_Form()
        self.ui.setupUi(self)

        # 创建一个Qt工作线程 (不可以为局部变量)
        self.thread = QtThread()

        self.slider_value_changed()

    def slider_value_changed(self):
        self.ui.Slider_X.valueChanged.connect(
            lambda: self.ui.label_X.setText("X:{}".format(self.ui.Slider_X.value())))

        self.ui.Slider_Y.valueChanged.connect(
            lambda: self.ui.label_Y.setText("Y:{}".format(self.ui.Slider_Y.value())))

        self.ui.Slider_Z.valueChanged.connect(
            lambda: self.ui.label_Z.setText("Z:{}".format(self.ui.Slider_Z.value())))

        self.ui.Slider_Rx.valueChanged.connect(
            lambda: self.ui.label_Rx.setText("Rx:{}".format(self.ui.Slider_Rx.value())))

        self.ui.Slider_Ry.valueChanged.connect(
            lambda: self.ui.label_Ry.setText("Ry:{}".format(self.ui.Slider_Ry.value())))

        self.ui.Slider_Rz.valueChanged.connect(
            lambda: self.ui.label_Rz.setText("Rz:{}".format(self.ui.Slider_Rz.value())))

        self.ui.Slider_joint1.valueChanged.connect(
            lambda: self.ui.label_joint1.setText("joint1:{}".format(self.ui.Slider_joint1.value())))

        self.ui.Slider_joint2.valueChanged.connect(
            lambda: self.ui.label_joint2.setText("joint2:{}".format(self.ui.Slider_joint2.value())))

        self.ui.Slider_joint3.valueChanged.connect(
            lambda: self.ui.label_joint3.setText("joint3:{}".format(self.ui.Slider_joint3.value())))

        self.ui.Slider_joint4.valueChanged.connect(
            lambda: self.ui.label_joint4.setText("joint4:{}".format(self.ui.Slider_joint4.value())))

        self.ui.Slider_joint5.valueChanged.connect(
            lambda: self.ui.label_joint5.setText("joint5:{}".format(self.ui.Slider_joint5.value())))

        self.ui.Slider_joint6.valueChanged.connect(
            lambda: self.ui.label_joint6.setText("joint6:{}".format(self.ui.Slider_joint6.value())))

    def init_data(self, WRIST_DEFAULT_POSITION, WRIST_DEFAULT_EULER_ANGLE, ARM_DEAFULT_JOINT_ANGLE):
        self.ui.Slider_X.setValue(WRIST_DEFAULT_POSITION[0])
        self.ui.Slider_Y.setValue(WRIST_DEFAULT_POSITION[1])
        self.ui.Slider_Z.setValue(WRIST_DEFAULT_POSITION[2])
        self.ui.Slider_Rx.setValue(WRIST_DEFAULT_EULER_ANGLE[0])
        self.ui.Slider_Ry.setValue(WRIST_DEFAULT_EULER_ANGLE[1])
        self.ui.Slider_Rz.setValue(WRIST_DEFAULT_EULER_ANGLE[2])
        self.ui.Slider_joint1.setValue(ARM_DEAFULT_JOINT_ANGLE[0])
        self.ui.Slider_joint2.setValue(ARM_DEAFULT_JOINT_ANGLE[1])
        self.ui.Slider_joint3.setValue(ARM_DEAFULT_JOINT_ANGLE[2])
        self.ui.Slider_joint4.setValue(ARM_DEAFULT_JOINT_ANGLE[3])
        self.ui.Slider_joint5.setValue(ARM_DEAFULT_JOINT_ANGLE[4])
        self.ui.Slider_joint6.setValue(ARM_DEAFULT_JOINT_ANGLE[5])

        self.ui.label_X.setText("X：{}".format(WRIST_DEFAULT_POSITION[0]))
        self.ui.label_Y.setText("Y：{}".format(WRIST_DEFAULT_POSITION[1]))
        self.ui.label_Z.setText("Z：{}".format(WRIST_DEFAULT_POSITION[2]))
        self.ui.label_Rx.setText("Rx：{}".format(WRIST_DEFAULT_EULER_ANGLE[0]))
        self.ui.label_Ry.setText("Ry：{}".format(WRIST_DEFAULT_EULER_ANGLE[1]))
        self.ui.label_Rz.setText("Rz：{}".format(WRIST_DEFAULT_EULER_ANGLE[2]))

        self.ui.label_joint1.setText("joint1：{}".format(ARM_DEAFULT_JOINT_ANGLE[0]))
        self.ui.label_joint2.setText("joint2：{}".format(ARM_DEAFULT_JOINT_ANGLE[1]))
        self.ui.label_joint3.setText("joint3：{}".format(ARM_DEAFULT_JOINT_ANGLE[2]))
        self.ui.label_joint4.setText("joint4：{}".format(ARM_DEAFULT_JOINT_ANGLE[3]))
        self.ui.label_joint5.setText("joint5：{}".format(ARM_DEAFULT_JOINT_ANGLE[4]))
        self.ui.label_joint6.setText("joint6：{}".format(ARM_DEAFULT_JOINT_ANGLE[5]))

    def sync_posture_data(self, posture):
        self.ui.Slider_X.setValue(posture[0])
        self.ui.Slider_Y.setValue(posture[1])
        self.ui.Slider_Z.setValue(posture[2])
        self.ui.Slider_Rx.setValue(posture[3])
        self.ui.Slider_Ry.setValue(posture[4])
        self.ui.Slider_Rz.setValue(posture[5])

        self.ui.label_X.setText("X：{}".format(posture[0]))
        self.ui.label_Y.setText("Y：{}".format(posture[1]))
        self.ui.label_Z.setText("Z：{}".format(posture[2]))
        self.ui.label_Rx.setText("Rx：{}".format(posture[3]))
        self.ui.label_Ry.setText("Ry：{}".format(posture[4]))
        self.ui.label_Rz.setText("Rz：{}".format(posture[5]))

    def sync_joint_data(self, angles):
        self.ui.Slider_joint1.setValue(angles[0])
        self.ui.Slider_joint2.setValue(angles[1])
        self.ui.Slider_joint3.setValue(angles[2])
        self.ui.Slider_joint4.setValue(angles[3])
        self.ui.Slider_joint5.setValue(angles[4])
        self.ui.Slider_joint6.setValue(angles[5])

        self.ui.label_joint1.setText("joint1：{}".format(angles[0]))
        self.ui.label_joint2.setText("joint1：{}".format(angles[1]))
        self.ui.label_joint3.setText("joint1：{}".format(angles[2]))
        self.ui.label_joint4.setText("joint1：{}".format(angles[3]))
        self.ui.label_joint5.setText("joint1：{}".format(angles[4]))
        self.ui.label_joint6.setText("joint1：{}".format(angles[5]))

    def read_data(self):
        data = dict()
        data.setdefault("posture", list())
        data["posture"].append(self.ui.Slider_X.value())
        data["posture"].append(self.ui.Slider_Y.value())
        data["posture"].append(self.ui.Slider_Z.value())
        data["posture"].append(self.ui.Slider_Rx.value())
        data["posture"].append(self.ui.Slider_Ry.value())
        data["posture"].append(self.ui.Slider_Rz.value())
        data.setdefault("angles", list())
        data["angles"].append(self.ui.Slider_joint1.value())
        data["angles"].append(self.ui.Slider_joint2.value())
        data["angles"].append(self.ui.Slider_joint3.value())
        data["angles"].append(self.ui.Slider_joint4.value())
        data["angles"].append(self.ui.Slider_joint5.value())
        data["angles"].append(self.ui.Slider_joint6.value())
        return data
