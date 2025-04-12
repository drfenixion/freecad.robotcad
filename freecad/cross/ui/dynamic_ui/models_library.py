import importlib
import os
from pathlib import Path
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
        self.main_layout = QtWidgets.QVBoxLayout()

        # prepare data
        from modules.robot_descriptions.robot_descriptions._descriptions import DESCRIPTIONS
        from modules.robot_descriptions.robot_descriptions._repositories import REPOSITORIES
        self.packages_grouped_by_tags = {}
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
                vendor = re.sub(r"face\Sook", '', vendor, flags=re.IGNORECASE)
                package_label = name.replace('_description', '').capitalize() + ' ' + vendor.capitalize()
                setattr(desc, 'package_label', package_label)
                setattr(desc, 'show', True)
                for tag in desc.tags:
                    if tag in self.packages_grouped_by_tags:
                        self.packages_grouped_by_tags[tag]['packages'].append(desc)
                    else:
                        self.packages_grouped_by_tags[tag] = {'show': True, 'packages':[desc]}


        self.display_filter_block()
        self.display_packages_block()

        self.button = QtWidgets.QPushButton('Open model variants')
        self.button.clicked.connect(self.get_selected_value)
        self.main_layout.addWidget(self.button)

        # # link to docks
        # weblink = QtWidgets.QLabel()
        # weblink.setText("<a href='https://github.com/drfenixion/robot_descriptions.py#descriptions'>https://github.com/drfenixion/robot_descriptions.py#descriptions</a>")
        # weblink.setTextFormat(QtCore.Qt.RichText)
        # weblink.setTextInteractionFlags(QtCore.Qt.TextBrowserInteraction)
        # weblink.setOpenExternalLinks(True)
        # self.main_layout.addWidget(QtWidgets.QLabel())
        # self.main_layout.addWidget(QtWidgets.QLabel())
        # self.main_layout.addWidget(weblink)

        # adding widgets to main layout
        self.setLayout(self.main_layout)
        self.show()


    def display_filter_block(self):

        def add_tag_button(layout, tag_name):
            radio_button = QtWidgets.QRadioButton(
                tag_name,
            )
            radio_button.clicked.connect(self.filter_by_tag)
            self.tags_radio_buttons.append(radio_button)
            self.tags_button_group.addButton(radio_button)
            layout.addWidget(radio_button, row_val, column_val)


        formGroupBox = QtWidgets.QGroupBox('Filter by tag')
        row_val = 0
        column_val = 0
        self.tags_radio_buttons = []
        self.tags_button_group = QtWidgets.QButtonGroup()
        layout = QtWidgets.QGridLayout()
        for tag_name, packages in self.packages_grouped_by_tags.items():
            add_tag_button(layout, tag_name)
            column_val += 1
            if column_val > 3:
                column_val = 0
                row_val += 1
        add_tag_button(layout, 'all')
        column_val += 1
        if column_val > 3:
            column_val = 0
            row_val += 1

        for i in range(layout.columnCount()):
            layout.setColumnStretch(i, 1)
        # for i in range(layout.rowCount()):
        #     layout.setRowStretch(i, 1)
        formGroupBox.setLayout(layout)
        self.main_layout.addWidget(formGroupBox)


    def display_packages_block(self):
        if hasattr(self, 'packagesFormGroupBox'):
            self.main_layout.removeWidget(self.packagesFormGroupBox)
            self.packagesFormGroupBox.deleteLater()
        self.packagesFormGroupBox = QtWidgets.QGroupBox('Models description packages')
        row_val = 0
        column_val = 0
        self.radio_buttons = []
        self.button_group = QtWidgets.QButtonGroup()
        layout = QtWidgets.QGridLayout()
        for tag_name, tag in self.packages_grouped_by_tags.items():
            for package in tag['packages']:
                if tag['show']:
                    radio_button = QtWidgets.QRadioButton(
                        package.package_label,
                    )
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
        self.packagesFormGroupBox.setLayout(layout)
        self.main_layout.insertWidget(1, self.packagesFormGroupBox)


    def filter_by_tag(self):
        for radio_button in self.tags_radio_buttons:
            if radio_button.isChecked():
                filter_tag_name = radio_button.text()
                for tag_name, tag in self.packages_grouped_by_tags.items():
                    if filter_tag_name == 'all':
                        self.packages_grouped_by_tags[tag_name]['show'] = True
                    else:
                        if filter_tag_name != tag_name:
                            self.packages_grouped_by_tags[tag_name]['show'] = False
                        else:
                            self.packages_grouped_by_tags[tag_name]['show'] = True
        self.display_packages_block()


    def get_selected_value(self):
        self.setEnabled(False)
        for radio_button in self.radio_buttons:
            if radio_button.isChecked():

                radio_button_text_first_fragment = radio_button.text().split()[0].lower()
                description_name = radio_button_text_first_fragment + '_description'
                description_name_alternative = radio_button_text_first_fragment

                progressBar = get_progress_bar(
                    title = "Cloning repository of " + description_name + "...",
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
                QtGui.QApplication.processEvents()

                # get urdf variants
                variants = {}
                for attr_name in dir(module):
                    if attr_name.startswith("URDF_PATH"):
                        attr_value = getattr(module, attr_name)
                        variants[attr_name + ' (' + Path(attr_value).name + ')'] = attr_value

                dialog = LoadURDFDialog(module, variants, parrent_window = self, package_name = radio_button.text())
                dialog.setModal(True)
                self.setEnabled(True)
                dialog.exec_()

                return
        QtWidgets.QMessageBox.warning(self, "Nothing selected", "Please select model to create.")


class LoadURDFDialog(QtWidgets.QDialog):
    def __init__(self, module, variants, parrent_window, package_name, parent=None):
        super(LoadURDFDialog, self).__init__(parent)
        self.module = module
        self.variants = variants
        self.parrent_window = parrent_window
        self.package_name = package_name
        self.create_without_solids = False
        self.initUI()


    def initUI(self):
        self.resize(400, 350)
        self.setWindowTitle("Variants of " + self.package_name)

        self.layout = QtWidgets.QVBoxLayout()
        self.layout.setContentsMargins(10, 10, 10, 10)
        self.setLayout(self.layout)

        self.radio_button_group = QtWidgets.QButtonGroup()
        self.radio_button_group.setExclusive(True)

        first = True
        for name, path in self.variants.items():
            radio_button = QtWidgets.QRadioButton(name)
            if first:
                radio_button.setChecked(True)
                first = False
            self.radio_button_group.addButton(radio_button)
            self.layout.addWidget(radio_button)

        self.create_without_solids_checkbox = QtWidgets.QCheckBox("Create without solids (fast but just for view)")
        self.create_without_solids_checkbox.stateChanged.connect(self.update_create_without_solids)
        self.layout.addWidget(self.create_without_solids_checkbox)

        self.layout.addSpacing(10)

        self.load_button = QtWidgets.QPushButton("Create model based on selection")
        self.load_button.clicked.connect(self.load_urdf)
        self.layout.addWidget(self.load_button)

        # Add a vertical spacer to push widgets up
        self.layout.addStretch()
        # Set the window to resize to fit its content
        self.adjustSize()


    def update_create_without_solids(self, state):
        if state == QtCore.Qt.Checked.value:
            self.create_without_solids = True
        else:
            self.create_without_solids = False


    def load_urdf(self):
        # get choosed variant
        selected_variant = None
        for radio_button in self.radio_button_group.buttons():
            if radio_button.isChecked():
                selected_variant = self.variants[radio_button.text()]
                break

        # disable buttons
        self.load_button.setEnabled(False)
        for rb in self.radio_button_group.buttons():
            rb.setEnabled(False)

        # Create model
        if selected_variant:
            robot_from_urdf_path(
                fc.activeDocument(),
                selected_variant,
                self.module.PACKAGE_PATH,
                self.module.REPOSITORY_PATH,
                create_without_solids=self.create_without_solids,
            )
            # enable buttons
            self.load_button.setEnabled(True)
            for rb in self.radio_button_group.buttons():
                rb.setEnabled(True)
        else:
            print("Model variant not selected")

        self.close()
