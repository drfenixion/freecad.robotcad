import importlib
import os
from pathlib import Path
import re
import sys
import threading
from PySide import QtGui, QtCore, QtWidgets
import FreeCAD as fc
from freecad.cross.freecad_utils import message
from freecad.cross.freecadgui_utils import get_progress_bar
from freecad.cross.robot_from_urdf import robot_from_urdf_path
from ...wb_utils import ROBOT_DESCRIPTIONS_MODULE_PATH, ROBOT_DESCRIPTIONS_REPO_PATH, git_init_submodules


class _CloneProgressEmitter(QtCore.QObject):
    """Thread-safe bridge for git clone progress updates.

    GitPython reports clone progress from background "pump" threads, so the
    value is forwarded to the widgets through a queued signal connection.
    """

    value_changed = QtCore.Signal(int)


def _get_robot_descriptions_cache_module():
    """Return the ``robot_descriptions._cache`` module used by descriptions.

    Description modules are loaded by file path under the top-level package
    name ``robot_descriptions``, so their relative ``from ._cache import ...``
    resolves to ``robot_descriptions._cache`` loaded from
    ``ROBOT_DESCRIPTIONS_MODULE_PATH``. This helper loads that exact module so
    that patching ``CloneProgressBar`` takes effect.
    """
    parent_name = 'robot_descriptions'
    parent_init = os.path.join(ROBOT_DESCRIPTIONS_MODULE_PATH, '__init__.py')
    parent_spec = importlib.util.spec_from_file_location(parent_name, parent_init)
    parent_module = importlib.util.module_from_spec(parent_spec)
    sys.modules[parent_name] = parent_module
    parent_spec.loader.exec_module(parent_module)

    cache_name = 'robot_descriptions._cache'
    cache_path = os.path.join(ROBOT_DESCRIPTIONS_MODULE_PATH, '_cache.py')
    cache_spec = importlib.util.spec_from_file_location(cache_name, cache_path)
    cache_module = importlib.util.module_from_spec(cache_spec)
    sys.modules[cache_name] = cache_module
    cache_spec.loader.exec_module(cache_module)
    return cache_module


def install_clone_progress_hook(progress_bar):
    """Route robot_descriptions git clone progress to a Qt progress bar.

    The repository download percentage is mapped to the 5-95% range of the
    given progress bar. Must be called before the robot description module is
    imported, because the clone happens at import time.

    Returns the emitter that must be kept alive by the caller for the duration
    of the clone.
    """
    rd_cache = _get_robot_descriptions_cache_module()

    emitter = _CloneProgressEmitter()
    # Use a queued connection so the progress bar is always updated in the GUI
    # thread, even though the signal is emitted from GitPython's worker threads.
    emitter.value_changed.connect(
        progress_bar.setValue,
        QtCore.Qt.QueuedConnection,
    )
    state = {'last': 5}

    class QtCloneProgressBar(rd_cache.RemoteProgress):
        """RemoteProgress that reports the download ratio to a Qt widget."""

        def update(self, op_code, cur_count, max_count=None, message=''):
            # Only follow the network download stage (RECEIVING). Other stages
            # (resolving deltas, checkout, ...) are ignored so the bar keeps
            # reflecting the download percentage.
            if not (op_code & self.RECEIVING):
                return
            if not max_count:
                return
            try:
                fraction = float(cur_count) / float(max_count)
            except (TypeError, ValueError, ZeroDivisionError):
                return
            value = max(5, min(95, 5 + int(fraction * 90)))
            if value != state['last']:
                state['last'] = value
                emitter.value_changed.emit(value)

    rd_cache.CloneProgressBar = QtCloneProgressBar
    return emitter


class ModelsLibraryModalClass(QtGui.QDialog):
    """ Display modal with models library """

    is_models_list_updated = False

    def __init__(self):
        super(ModelsLibraryModalClass, self).__init__()

        git_init_submodules(
            submodule_repo_path = ROBOT_DESCRIPTIONS_REPO_PATH,
        )
        self.initUI()


    def initUI(self):
        # Size the dialog to 75% of the FreeCAD main window.
        preferred_width = 800
        preferred_height = 1000
        import FreeCADGui as fcg
        main_window = fcg.getMainWindow()
        if main_window is not None:
            preferred_width = max(200, main_window.width() * 3 // 4)
            preferred_height = max(200, main_window.height() * 3 // 4)
        self.resize(preferred_width, preferred_height)
        self.setWindowTitle("Models library")

        # The whole content lives inside a scroll area so the models list can
        # be scrolled vertically when it does not fit into the window height.
        self.content_widget = QtWidgets.QWidget()
        self.main_layout = QtWidgets.QVBoxLayout()
        self.main_layout.setContentsMargins(10, 10, 10, 10)
        self.content_widget.setLayout(self.main_layout)

        self.scroll_area = QtWidgets.QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.scroll_area.setWidget(self.content_widget)

        # Root layout: scrollable content on top, action buttons fixed below.
        self.root_layout = QtWidgets.QVBoxLayout()
        self.root_layout.setContentsMargins(10, 10, 10, 10)
        self.root_layout.addWidget(self.scroll_area, 1)

        # prepare data
        from modules.robot_descriptions.robot_descriptions._descriptions import DESCRIPTIONS
        from modules.robot_descriptions.robot_descriptions._repositories import REPOSITORIES
        self.packages_grouped_by_tags = {}
        for name in sorted(list(DESCRIPTIONS)):
            desc = DESCRIPTIONS[name]
            if desc.has_urdf or desc.has_mjcf:

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

                        # select vendor
                        match = re.search(r"https://github.com/(.*)/", repository.url)
                        if match:
                            vendor = match.group(1)

                #QtWidgets.QLabel()
                vendor = re.sub(r"face\Sook", '', vendor, flags=re.IGNORECASE)
                package_label = name.replace('_description', '').capitalize() + ' ' + vendor.capitalize()
                package = {'desc': desc, 'package_label': package_label}
                for tag in desc.tags:
                    if tag in self.packages_grouped_by_tags:
                        self.packages_grouped_by_tags[tag]['packages'].append(package)
                    else:
                        self.packages_grouped_by_tags[tag] = {'show': True, 'packages':[package]}


        self.display_filter_block()
        self.display_packages_block()

        self.button = QtWidgets.QPushButton('Open model variants')
        self.button.clicked.connect(self.get_selected_value)
        self.root_layout.addWidget(self.button)

        self.update_models_list_button = QtWidgets.QPushButton('Update models list')
        self.update_models_list_button.clicked.connect(self.update_models_list)
        if self.__class__.is_models_list_updated:
            self.update_models_list_button.setEnabled(False)
        self.root_layout.addWidget(self.update_models_list_button)

        # link to docks
        weblink = QtWidgets.QLabel()
        weblink.setText("<a href='https://github.com/robot-descriptions/robot_descriptions.py#descriptions'>https://github.com/robot-descriptions/robot_descriptions.py#descriptions</a> for manually adding your model do PR. You can also check the licenses of the models there.")
        weblink.setTextFormat(QtCore.Qt.RichText)
        weblink.setTextInteractionFlags(QtCore.Qt.TextBrowserInteraction)
        weblink.setOpenExternalLinks(True)
        self.main_layout.addWidget(QtWidgets.QLabel())
        self.main_layout.addWidget(QtWidgets.QLabel())
        self.main_layout.addWidget(weblink)

        # description
        self.main_layout.addWidget(QtWidgets.QLabel())
        description = QtWidgets.QLabel()
        description.setText("To raise your models to the top of the section, write to <a href='mailto:it.project.devel@gmail.com'>it.project.devel@gmail.com</a>. It is also possible to add your models as a service.")
        description.setTextFormat(QtCore.Qt.RichText)
        description.setTextInteractionFlags(QtCore.Qt.TextBrowserInteraction)
        description.setOpenExternalLinks(True)
        self.main_layout.addWidget(description)

        # description
        self.main_layout.addWidget(QtWidgets.QLabel())
        description = QtWidgets.QLabel()
        description.setText("If faced crash check free RAM or swap. Some models can take 10 or more minutes and 10Gb free RAM to create.")
        self.main_layout.addWidget(description)

        # description
        self.main_layout.addWidget(QtWidgets.QLabel())
        description = QtWidgets.QLabel()
        description.setText("Use models without creating solids only for fast view. Solids are needed for ineartia/mass calculation, placement tools, collisions adding, etc.")
        self.main_layout.addWidget(description)

        # adding widgets to root layout
        self.setLayout(self.root_layout)
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
                        package['package_label'],
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


    def update_models_list(self):
        self.setEnabled(False)
        git_init_submodules(
            only_first_update = False,
            submodule_repo_path = ROBOT_DESCRIPTIONS_REPO_PATH,
        )
        message("Models list updated", True)
        self.close()
        self.deleteLater()
        self.__class__.is_models_list_updated = True
        form = ModelsLibraryModalClass()
        form.exec_()


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

                # Show 5% immediately, then follow the actual repository
                # download percentage reported by the git clone.
                progressBar.setValue(5)
                self._clone_progress_emitter = install_clone_progress_hook(progressBar)
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

                def load_robot_desc_module():
                    """Import the description module (this clones the repo).

                    Runs in a worker thread, so the git clone progress is
                    reported while the GUI thread keeps processing events.
                    """
                    # override global robot_descriptions module if present
                    import_robot_desc_module_by_path(
                        f"robot_descriptions",
                        os.path.join(ROBOT_DESCRIPTIONS_MODULE_PATH, f'__init__.py'),
                    )
                    module_path = os.path.join(ROBOT_DESCRIPTIONS_MODULE_PATH, f'{description_name}.py')
                    module_path_alternative = os.path.join(ROBOT_DESCRIPTIONS_MODULE_PATH, f'{description_name_alternative}.py')
                    try:
                        return import_robot_desc_module_by_path(
                            f"robot_descriptions.{description_name}",
                            module_path,
                        )
                    except FileNotFoundError:
                        try:
                            return import_robot_desc_module_by_path(
                                f"robot_descriptions.{description_name_alternative}",
                                module_path,
                            )
                        except FileNotFoundError:
                            return import_robot_desc_module_by_path(
                                f"robot_descriptions.{description_name_alternative}",
                                module_path_alternative,
                            )

                # Run the blocking clone/import in a worker thread and keep
                # pumping GUI events here so the queued progress updates are
                # applied while the repository is being downloaded.
                load_result = {}
                load_error = {}

                def _load_target():
                    try:
                        load_result['module'] = load_robot_desc_module()
                    except Exception as error:  # noqa: BLE001
                        load_error['error'] = error

                load_thread = threading.Thread(target=_load_target, daemon=True)
                load_thread.start()
                while load_thread.is_alive():
                    QtGui.QApplication.processEvents()
                    load_thread.join(0.05)

                if 'error' in load_error:
                    progressBar.close()
                    self.setEnabled(True)
                    raise load_error['error']
                module = load_result['module']

                progressBar.setValue(100)
                QtGui.QApplication.processEvents()
                progressBar.close()
                QtGui.QApplication.processEvents()

                # get urdf variants
                variants = {}
                for attr_name in dir(module):
                    if attr_name.startswith("URDF_PATH"):
                        attr_value = getattr(module, attr_name)
                        variants[attr_name + ' (' + Path(attr_value).name + ')'] = {
                            'path': attr_value,
                            'is_xacro': False,
                            'xacro_args': None,
                            'is_mjcf': False,
                        }
                    elif attr_name.startswith("XACRO_PATH"):
                        attr_value = getattr(module, attr_name)
                        variants[attr_name + ' (' + Path(attr_value).name + ')'] = {
                            'path': attr_value,
                            'is_xacro': True,
                            'xacro_args': None,
                            'is_mjcf': False,
                        }
                    elif attr_name.startswith("MJCF_PATH"):
                        attr_value = getattr(module, attr_name)
                        variants[attr_name + ' (' + Path(attr_value).name + ')'] = {
                            'path': attr_value,
                            'is_xacro': False,
                            'xacro_args': None,
                            'is_mjcf': True,
                        }

                # Add variants for additional XACRO_ARGS_* argument sets
                # (e.g. XACRO_ARGS_NO_HAND, XACRO_ARGS_LEFT_ARM)
                if hasattr(module, "XACRO_PATH"):
                    for attr_name in dir(module):
                        if attr_name.startswith("XACRO_ARGS") and attr_name != "XACRO_ARGS":
                            xacro_args = getattr(module, attr_name)
                            if isinstance(xacro_args, dict):
                                variant_suffix = attr_name[len("XACRO_ARGS"):].strip('_').replace('_', ' ')
                                variant_label = f"XACRO_PATH ({Path(module.XACRO_PATH).name}) [{variant_suffix}]"
                                variants[variant_label] = {
                                    'path': module.XACRO_PATH,
                                    'is_xacro': True,
                                    'xacro_args': xacro_args,
                                    'is_mjcf': False,
                                }

                dialog = LoadURDFDialog(module, variants, parrent_window = self, package_name = radio_button.text())
                dialog.setModal(True)
                self.setEnabled(True)
                dialog.exec_()

                return
        self.setEnabled(True)
        QtWidgets.QMessageBox.warning(self, "Nothing selected", "Please select model to create.")


class LoadURDFDialog(QtWidgets.QDialog):
    def __init__(self, module, variants, parrent_window, package_name, parent=None):
        super(LoadURDFDialog, self).__init__(parent)
        self.module = module
        self.variants = variants
        self.parrent_window = parrent_window
        self.package_name = package_name
        self.create_without_solids = True
        self.remove_solid_splitter = True
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
        for name, variant in self.variants.items():
            radio_button = QtWidgets.QRadioButton(name)
            if first:
                radio_button.setChecked(True)
                first = False
            self.radio_button_group.addButton(radio_button)
            self.layout.addWidget(radio_button)

        self.create_without_solids_checkbox = QtWidgets.QCheckBox("Don`t create solids (only fast view)")
        self.create_without_solids_checkbox.setChecked(self.create_without_solids)
        self.create_without_solids_checkbox.stateChanged.connect(self.update_create_without_solids)
        self.layout.addWidget(self.create_without_solids_checkbox)

        self.remove_solid_splitter_checkbox = QtWidgets.QCheckBox("Remove splitters (edges) from solids (usefull for Set Placement)")
        if self.remove_solid_splitter:
            self.remove_solid_splitter_checkbox.setChecked(True)
        self.remove_solid_splitter_checkbox.stateChanged.connect(self.update_remove_solid_splitter)
        self.layout.addWidget(self.remove_solid_splitter_checkbox)

        self.layout.addSpacing(10)

        self.load_button = QtWidgets.QPushButton("Create model")
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


    def update_remove_solid_splitter(self, state):
        if state == QtCore.Qt.Checked.value:
            self.remove_solid_splitter = True
        else:
            self.remove_solid_splitter = False


    def load_urdf(self):
        # get choosed variant
        selected_variant = None
        for radio_button in self.radio_button_group.buttons():
            if radio_button.isChecked():
                selected_variant = self.variants[radio_button.text()]
                break

        # disable buttons
        self.setEnabled(False)

        # Create model
        if selected_variant:
            urdf_path = selected_variant['path']
            if selected_variant['is_xacro']:
                # Convert xacro to URDF using robot_descriptions._xacro
                # (which uses xacrodoc) and process it by the URDF scenario.
                from robot_descriptions._xacro import get_urdf_path as get_urdf_path_from_xacro
                urdf_path = get_urdf_path_from_xacro(
                    self.module,
                    xacro_args=selected_variant['xacro_args'],
                )
            elif selected_variant['is_mjcf']:
                # Convert MJCF to URDF using the MuJoCo converter and process it
                # by the URDF scenario.
                from freecad.cross.mjcf_utils import get_urdf_path as get_urdf_path_from_mjcf
                urdf_path = get_urdf_path_from_mjcf(
                    selected_variant['path'],
                    package_path=self.module.PACKAGE_PATH,
                    repository_path=self.module.REPOSITORY_PATH,
                )
            robot_from_urdf_path(
                fc.activeDocument(),
                urdf_path,
                self.module.PACKAGE_PATH,
                self.module.REPOSITORY_PATH,
                create_without_solids=self.create_without_solids,
                remove_solid_splitter=self.remove_solid_splitter,
            )
            # enable buttons
            self.setEnabled(True)
        else:
            print("Model variant not selected")

        self.close()
