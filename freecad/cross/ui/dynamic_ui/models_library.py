import importlib
import os
import re
import sys
from PySide import QtGui, QtCore, QtWidgets
import FreeCAD as fc
from freecad.cross.freecadgui_utils import get_progress_bar
from freecad.cross.robot_from_urdf import robot_from_urdf_path
from ...wb_utils import ROBOT_DESCRIPTIONS_MODULE_PATH, ROBOT_DESCRIPTIONS_REPO_PATH, git_init_submodules


class ModelsLibraryModalClass(QtGui.QDialog):
    """ Display modal with models library """

    def __init__(self):
        super(ModelsLibraryModalClass, self).__init__()

        git_init_submodules(
            submodule_repo_path = ROBOT_DESCRIPTIONS_REPO_PATH,
        )
        self.initUI()


    def initUI(self):
        self.resize(400, 350)
        self.setWindowTitle("Models library")

        main_layout = QtWidgets.QVBoxLayout()
        layout = {}
        formGroupBox = {}

        # block
        formGroupBox = QtWidgets.QGroupBox('Models description packages')
        layout = QtWidgets.QGridLayout()
        self.radio_buttons = []
        self.button_group = QtWidgets.QButtonGroup()
        row_val = 0
        column_val = 0
        from modules.robot_descriptions.robot_descriptions._descriptions import DESCRIPTIONS
        from modules.robot_descriptions.robot_descriptions._repositories import REPOSITORIES
        for name in sorted(list(DESCRIPTIONS)):
            desc = DESCRIPTIONS[name]
            if desc.has_urdf:

                vendor = ''
                #get module code for parse
                # spec = importlib.util.find_spec(f"robot_descriptions.{name}") #in case of direct pip module import
                spec = importlib.util.spec_from_file_location(
                    f"robot_descriptions.{name}",
                    os.path.join(ROBOT_DESCRIPTIONS_MODULE_PATH, f'{name}.py'),
                )
                if spec and spec.origin:
                    module_path = spec.origin
                    with open(module_path, 'r') as file:
                        module_code = file.read()

                    #select repository name
                    match = re.search(r'_clone_to_cache\([\n\r\s]*"(.*?)",', module_code, re.MULTILINE)

                    if match:
                        repository_name = match.group(1)
                        repository = REPOSITORIES[repository_name]

                        # select sendor
                        match = re.search(r"https://github.com/(.*)/", repository.url)
                        if match:
                            vendor = match.group(1)

                #QtWidgets.QLabel()
                radio_button = QtWidgets.QRadioButton(name.replace('_description', '').capitalize() + ' ' + vendor.capitalize())
                self.radio_buttons.append(radio_button)
                self.button_group.addButton(radio_button)
                layout.addWidget(radio_button, row_val, column_val)
                column_val += 1
                if column_val > 3:
                    column_val = 0
                    row_val += 1

        for i in range(layout.columnCount()):
            layout.setColumnStretch(i, 1)

        for i in range(layout.rowCount()):
            layout.setRowStretch(i, 1)

        formGroupBox.setLayout(layout)
        main_layout.addWidget(formGroupBox)

        self.button = QtWidgets.QPushButton('Create model based on selection')
        self.button.clicked.connect(self.get_selected_value)
        main_layout.addWidget(self.button)

        # link to docks
        weblink = QtWidgets.QLabel()
        weblink.setText("<a href='https://github.com/drfenixion/robot_descriptions.py#descriptions'>https://github.com/drfenixion/robot_descriptions.py#descriptions</a>")
        weblink.setTextFormat(QtCore.Qt.RichText)
        weblink.setTextInteractionFlags(QtCore.Qt.TextBrowserInteraction)
        weblink.setOpenExternalLinks(True)
        main_layout.addWidget(QtWidgets.QLabel())
        main_layout.addWidget(QtWidgets.QLabel())
        main_layout.addWidget(weblink)

        # adding widgets to main layout
        self.setLayout(main_layout)

        self.show()


    def get_selected_value(self):
        for radio_button in self.radio_buttons:
            if radio_button.isChecked():

                self.button.setEnabled(False)
                for rb in self.radio_buttons:
                    rb.setEnabled(False)

                radio_button_text_first_fragment = radio_button.text().split()[0].lower()
                description_name = radio_button_text_first_fragment + '_description'
                description_name_alternative = radio_button_text_first_fragment

                progressBar = get_progress_bar(
                    title = "Cloning repository of " + description_name + "... Please wait.",
                    min = 0,
                    max = 100,
                    show_percents = False,
                )
                progressBar.show()

                i = 0
                progressBar.setValue(i)
                QtGui.QApplication.processEvents()

                #module = import_module(f"robot_descriptions.{description_name}") #in case of direct pip module import
                def import_robot_desc_module_by_path(module_name, module_path):
                    spec = importlib.util.spec_from_file_location(
                        module_name,
                        module_path,
                    )
                    module = importlib.util.module_from_spec(spec)
                    sys.modules[module_name] = module
                    spec.loader.exec_module(module)

                    return module
                # override global robot_descriptions module if present
                import_robot_desc_module_by_path(
                    f"robot_descriptions",
                    os.path.join(ROBOT_DESCRIPTIONS_MODULE_PATH, f'__init__.py'),
                )
                module_path = os.path.join(ROBOT_DESCRIPTIONS_MODULE_PATH, f'{description_name}.py')
                try:
                    module = import_robot_desc_module_by_path(
                        f"robot_descriptions.{description_name}",
                        module_path,
                    )
                except FileNotFoundError:
                    module = import_robot_desc_module_by_path(
                        f"robot_descriptions.{description_name_alternative}",
                        module_path,
                    )
                i = 100
                progressBar.setValue(i)
                QtGui.QApplication.processEvents()
                progressBar.close()
                robot_from_urdf_path(
                    fc.activeDocument(),
                    module.URDF_PATH,
                    module.PACKAGE_PATH,
                    module.REPOSITORY_PATH,
                )

                for rb in self.radio_buttons:
                    rb.setEnabled(True)
                self.button.setEnabled(True)

                return
        QtWidgets.QMessageBox.warning(self, "Nothing selected", "Please select model to create.")
