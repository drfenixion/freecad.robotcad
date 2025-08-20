from xml.dom.minidom import parseString, Document

from freecad.cross.wb_gui_utils import WbSettingsGetter
from ..wb_utils import get_rel_and_abs_path, set_workbench_param
import FreeCAD as fc
import FreeCADGui as fcgui
from ..wb_utils import get_urdf_path
from ..wb_utils import get_xacro_wrapper_path
from ..wb_utils import get_robot_meta_path
from ..wb_utils import get_controllers_config_path
from ..gui_utils import tr
from ..wb_utils import is_robot_selected
from pathlib import Path
from ..freecad_utils import error
from io import BytesIO
import shutil
import os
import zipfile
import requests
from ..wb_utils import get_workbench_param
from .. import wb_globals
from ..ros.utils import split_package_path


class _TransferProjectToExternalCodeGeneratorCommand:
    """Command that transfers project to external code generator."""

    def GetResources(self):
        return {
            'Pixmap': 'urdf_export_external_generator.svg',
            'MenuText': tr('Extended external code generator'),
            'ToolTip': tr(
                'Select Robot and press this tool.\n'
                'Tool will locally generate ROS package files\n'
                'and transfer it to external code generator service and save gotten result files locally.\n'
                '\n'
                'Tool can generate:\n'
                '  - project structure (folders, git)\n'
                '  - specific robot types code (PX4 multicopters)\n'
                '  - ros2_controllers \n'
                '  - infrastructure: \n'
                '    - startup script code (one command to build and run docker with all dependencies of project) \n'
                '    - docker related code (dockerfile, etc)\n'
                '  - Required modification of basic ROS2 package:\n'
                '     - URDF\n'
                '     - ROS 2 package files\n'
                '     - etc.\n'
                '\n'
                'Provided by robotcad.ru',
            ),
        }


    def Activated(self):

      settings_getter = WbSettingsGetter()
      if not settings_getter.overcross_token:
        if settings_getter.get_settings(get_ros_workspace=False, get_vhacd_path=False, get_overcross_token=True):
          set_workbench_param(wb_globals.PREF_OVERCROSS_TOKEN, str(settings_getter.overcross_token))      

      # print('Calculating mass and inertia started.')
      # fcgui.runCommand("CalculateMassAndInertia")
      # print('Calculating mass and inertia finished.')

      print('Local code generating started.')
      # ensure files present by regenerate them
      fcgui.runCommand("UrdfExport")
      print('Local code generating finished.')

      print('Preparing files started')
      robot = self.getSelectedRobot()
      rel_output_path, abs_output_path = get_rel_and_abs_path(robot.OutputPath, ask_user_fill_workspace=False)

      project_path, package_name, description_package_path = split_package_path(abs_output_path)
      urdf_path = get_urdf_path(robot, description_package_path)
      xacro_wrapper_path = get_xacro_wrapper_path(robot, description_package_path)

      path_to_overcross_meta_dir_rel_to_project = 'overcross/'
      path_to_overcross_meta_dir = str(project_path) + '/' + path_to_overcross_meta_dir_rel_to_project

      path_to_robot_meta = get_robot_meta_path(path_to_overcross_meta_dir)
      path_to_controllers_config = get_controllers_config_path(robot, path_to_overcross_meta_dir)

      path_to_overcross_meta_tmp_dir = path_to_overcross_meta_dir + 'tmp/'

      # check urdf
      if not urdf_path.exists():
        error(f"File: {urdf_path} does not exists.", True)
        return

      urdf_content = urdf_path.read_text(encoding='utf-8', errors=None)

      if not urdf_content:
        error(f"File: {urdf_path} cannot be read or is empty.", True)
        return

      # check robot_meta
      if not path_to_robot_meta.exists():
        error(f"File: {path_to_robot_meta} does not exists.", True)
        return

      robot_meta_content = path_to_robot_meta.read_text(encoding='utf-8', errors=None)

      if not robot_meta_content:
        error(f"File: {path_to_robot_meta} cannot be read or is empty.", True)
        return

      # copy needed files to tmp dir for archive
      path_to_overcross_meta_dir_in_tmp_dir = path_to_overcross_meta_tmp_dir + path_to_overcross_meta_dir_rel_to_project
      Path(path_to_overcross_meta_dir_in_tmp_dir).mkdir(parents=True, exist_ok=True)
      shutil.copy(path_to_robot_meta, path_to_overcross_meta_dir_in_tmp_dir)
      shutil.copy(path_to_controllers_config, path_to_overcross_meta_dir_in_tmp_dir)

      path_to_overcross_urdf_dir_in_tmp_dir = path_to_overcross_meta_tmp_dir + '/' + package_name + '/' + 'urdf'
      Path(path_to_overcross_urdf_dir_in_tmp_dir).mkdir(parents=True, exist_ok=True)
      shutil.copy(urdf_path, path_to_overcross_urdf_dir_in_tmp_dir)
      shutil.copy(xacro_wrapper_path, path_to_overcross_urdf_dir_in_tmp_dir)

      # make archive with needed files
      zip_filename = "project"
      zip_filename_with_ext = zip_filename + '.zip'
      path_to_overcross_project_zip_in_meta_dir = path_to_overcross_meta_dir + zip_filename_with_ext
      self.saveArchive(path_to_overcross_meta_tmp_dir, path_to_overcross_meta_dir + zip_filename)

      token = get_workbench_param(wb_globals.PREF_OVERCROSS_TOKEN, '')
      print('Preparing files finished')

      print('External code generating started.')
      if os.environ.get('DEBUG'):
        # send file to external code generator
        print('DEBUG is active. Connect to localhost generator.')
        with open(path_to_overcross_project_zip_in_meta_dir, 'rb') as f:
          r = requests.post(
              'https://localhost/ru/generator/',
              data={'token': token},
              files={'file': f},
              allow_redirects=True,
              verify=False,
          )
      else:
        # send file to external code generator
        with open(path_to_overcross_project_zip_in_meta_dir, 'rb') as f:
          r = requests.post(
              'https://robotcad.ru/ru/generator/',
              data={'token': token},
              files={'file': f},
              allow_redirects=True,
          )

      if r.status_code == 402:
        error(r.text, True)
        # TODO redirect to tariffs page
        return

      if r.status_code == 403 or r.status_code == 401:
        error(r.text, True)
        fcgui.runCommand("WbSettings")
        return

      if r.status_code == 429:
        error(r.text, True)
        return

      if r.status_code == 500:
        error('External code generator server error. Try later.', True)
        return

      if r.status_code != 200:
        error('Server error. Try later.', True)
        return

      # write gotten generated archive
      path_to_generated_project_zip = path_to_overcross_meta_dir + "project_from_external_generator.zip"
      Path(path_to_generated_project_zip).write_bytes(r.content)

      # unarchive and place generated files to project
      with zipfile.ZipFile(path_to_generated_project_zip, 'r') as zip_ref:
        zip_ref.extractall(project_path)

      # delete zip`s and tmp dir
      Path(path_to_overcross_project_zip_in_meta_dir).unlink()
      Path(path_to_generated_project_zip).unlink()
      shutil.rmtree(path_to_overcross_meta_tmp_dir)

      print('External code generating finished. Files are saved locally.')



    def saveArchive(self, path: str, zip_filename = "project") -> None:

      shutil.make_archive(
          zip_filename, format='zip',
          root_dir=path,
      )


    def IsActive(self):
      return is_robot_selected()


    def getSelectedRobot(self):
      robot = fcgui.Selection.getSelectionEx()[0].Object

      return robot


fcgui.addCommand('TransferProjectToExternalCodeGenerator', _TransferProjectToExternalCodeGeneratorCommand())
