# import statements
from PySide import QtGui, QtCore, QtWidgets
import FreeCADGui as fcgui
import FreeCAD as fc
from functools import partial

from ...sensors.sensor_proxy import get_sensors_data
from ...wb_utils import is_robot_selected
from ...wb_utils import is_link_selected
from ...wb_utils import is_joint_selected
from ...sensors.sensor_factory import make_sensor
from ...freecad_utils import message


class SensorsSelectorModalClass(QtGui.QDialog):
    """ Display modal with sensors adder """

    def __init__(self):
        super(SensorsSelectorModalClass, self).__init__()
        self.initUI()


    def initUI(self):
        self.resize(400, 350)
        self.setWindowTitle("Select sensor")

        # get sensor data and make dropdowns
        self.sensors = get_sensors_data()

        main_layout = QtWidgets.QVBoxLayout()
        form_layout = {}
        formGroupBox = {}
        addSensorButton = {}
        self.sensors_dropdown = {}
        self.sensor_description = {}

        for sensor_dir_name, sensor_dir in self.sensors.items():
            # block
            formGroupBox[sensor_dir_name] = QtWidgets.QGroupBox('Attachable to ' + sensor_dir_name)
            form_layout[sensor_dir_name] = QtWidgets.QFormLayout(self)

            #dropdown
            self.sensors_dropdown[sensor_dir_name] = QtGui.QComboBox(self)
            for sensor_name, sensor_data in sensor_dir.items():
                self.sensors_dropdown[sensor_dir_name].addItem(sensor_name, sensor_data['description'])

            # sensors block
            self.sensor_description[sensor_dir_name] = QtWidgets.QLabel()
            form_layout[sensor_dir_name].addRow(QtWidgets.QLabel("Select sensor:"), self.sensors_dropdown[sensor_dir_name])

            # add sensors adding button
            addSensorButton[sensor_dir_name] = QtGui.QPushButton('Add sensor to ' + sensor_dir_name, self)

            addSensorButton[sensor_dir_name].clicked.connect(partial(self.onAddSensorButton, sensor_dir_name))
            addSensorButton[sensor_dir_name].setAutoDefault(False)
            form_layout[sensor_dir_name].addRow(QtWidgets.QLabel(""), addSensorButton[sensor_dir_name])

            # add to main layout
            formGroupBox[sensor_dir_name].setLayout(form_layout[sensor_dir_name])
            main_layout.addWidget(formGroupBox[sensor_dir_name])

        # Info
        info = QtWidgets.QLabel()
        info.setText("Info: sensors are positive x-oriented of parent joint (red arrow - the direction the sensor is looking)")
        main_layout.addWidget(info)

        # link to docks
        weblink = QtWidgets.QLabel()
        weblink.setText("<a href='https://gazebosim.org/docs/latest/sensors/'>Gazebo sensors docs</a>")
        weblink.setTextFormat(QtCore.Qt.RichText)
        weblink.setTextInteractionFlags(QtCore.Qt.TextBrowserInteraction)
        weblink.setOpenExternalLinks(True)
        main_layout.addWidget(weblink)

        # adding widgets to main layout
        self.setLayout(main_layout)

        self.show()


    def update_sensor_description(self, text, sensor_dir_name):
        index = self.sensors_dropdown[sensor_dir_name].findText(text)
        description = self.sensors_dropdown[sensor_dir_name].itemData(index)
        self.sensor_description[sensor_dir_name].setText(description)


    def onAddSensorButton(self, sensor_dir_name):
        if is_robot_selected or is_link_selected() or is_joint_selected():
            doc = fc.activeDocument()
            doc.openTransaction('Add Sensor')

            name = str(self.sensors_dropdown[sensor_dir_name].currentText())
            sensor = make_sensor(self.sensors[sensor_dir_name][name], sensor_dir_name)
            sel = fcgui.Selection.getSelection()[0]
            sel.addObject(sensor)

            doc.commitTransaction()
            doc.recompute()
        else:
            message('Select link or joint first', gui = True)
